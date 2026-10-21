/* Read XRandR capability; change only a uniquely titled, PID/start-owned Xephyr window. */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/stat.h>
#include <X11/Xlib.h>
#include <X11/extensions/Xrandr.h>
/* Public libXres 1.2 ABI; declarations keep runtime independent of libxres-dev.
This package was downloaded from
http://xorg.freedesktop.org/releases/individual/lib/

   Copyright (c) 2002  XFree86 Inc
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is fur-
nished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FIT-
NESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
XFREE86 PROJECT BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CON-
NECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Except as contained in this notice, the name of the XFree86 Project shall not
be used in advertising or otherwise to promote the sale, use or other deal-
ings in this Software without prior written authorization from the XFree86
Project.

*/
typedef struct { XID client; unsigned int mask; } XResClientIdSpec;
typedef struct { XResClientIdSpec spec; long length; void *value; } XResClientIdValue;
extern Bool XResQueryExtension(Display *, int *, int *);
extern Status XResQueryVersion(Display *, int *, int *);
extern Status XResQueryClientIds(Display *, long, XResClientIdSpec *, long *, XResClientIdValue **);
extern pid_t XResGetClientPid(XResClientIdValue *);
extern void XResClientIdsDestroy(long, XResClientIdValue *);
static int x_error;
static int record_error(Display *d, XErrorEvent *e) { (void)d; (void)e; x_error = 1; return 0; }
static void json_string(const char *s) {
    putchar('"');
    for (const unsigned char *p = (const unsigned char *)s; *p; ++p) {
        if (*p == '"' || *p == '\\') { putchar('\\'); putchar(*p); }
        else if (*p < 32) printf("\\u%04x", *p);
        else putchar(*p);
    }
    putchar('"');
}
static int process_matches(long pid, const char *started) {
    char path[128], text[8192], executable[4096];
    struct stat st;
    if (pid <= 0) return 0;
    snprintf(path, sizeof(path), "/proc/%ld", pid);
    if (stat(path, &st) || st.st_uid != geteuid()) return 0;
    snprintf(path, sizeof(path), "/proc/%ld/exe", pid);
    ssize_t n = readlink(path, executable, sizeof(executable) - 1);
    if (n < 0) return 0;
    executable[n] = 0;
    if (strcmp(executable, "/usr/bin/Xephyr")) return 0;
    snprintf(path, sizeof(path), "/proc/%ld/stat", pid);
    FILE *f = fopen(path, "r");
    if (!f) return 0;
    int read_ok = fgets(text, sizeof(text), f) != NULL;
    fclose(f);
    if (!read_ok) return 0;
    char *tail = strrchr(text, ')'), *save = NULL, *field;
    if (!tail) return 0;
    int index = 0;
    for (field = strtok_r(tail + 1, " \n", &save); field; field = strtok_r(NULL, " \n", &save), ++index) {
        if (index == 0 && !strcmp(field, "Z")) return 0;
        if (index == 19) return !strcmp(field, started);
    }
    return 0;
}
static long window_pid(Display *d, Window w) {
    int event, error, major = 0, minor = 0;
    if (!XResQueryExtension(d, &event, &error) || !XResQueryVersion(d, &major, &minor)
        || major < 1 || (major == 1 && minor < 2)) return -1;
    XResClientIdSpec spec = { w, 2 };
    XResClientIdValue *values = NULL;
    long count = 0, result = -1;
    if (XResQueryClientIds(d, 1, &spec, &count, &values) == Success) {
        for (long i = 0; i < count; ++i) {
            pid_t pid = XResGetClientPid(&values[i]);
            if (pid > 0) { result = pid; break; }
        }
    }
    if (values) XResClientIdsDestroy(count, values);
    return result;
}
static void find_title(Display *d, Window parent, const char *title, int depth,
                       int *scanned, int *matches, Window *found) {
    if (depth > 64 || *scanned >= 8192) { x_error = 1; return; }
    ++*scanned;
    char *name = NULL;
    if (XFetchName(d, parent, &name) && name && !strcmp(name, title)) { ++*matches; *found = parent; }
    if (name) XFree(name);
    Window root, actual_parent, *children = NULL;
    unsigned int count = 0;
    if (XQueryTree(d, parent, &root, &actual_parent, &children, &count)) {
        for (unsigned int i = 0; i < count; ++i)
            find_title(d, children[i], title, depth + 1, scanned, matches, found);
    }
    if (children) XFree(children);
}
static int query_modes(Display *d) {
    int event, error, count = 0, current = -1, current_rate = -1;
    Rotation rotation = 0;
    int extension = XRRQueryExtension(d, &event, &error);
    XRRScreenConfiguration *config = extension ? XRRGetScreenInfo(d, DefaultRootWindow(d)) : NULL;
    XRRScreenSize *sizes = NULL;
    if (config) {
        sizes = XRRConfigSizes(config, &count);
        current = XRRConfigCurrentConfiguration(config, &rotation);
        current_rate = XRRConfigCurrentRate(config);
    }
    printf("{\"display_open\":true,\"display\":"); json_string(DisplayString(d));
    printf(",\"randr_extension\":%s,\"screen_config_available\":%s,\"mode_count\":%d,"
           "\"mode_pointer_nonnull\":%s,\"current_mode_index\":%d,\"current_rate\":%d,\"modes\":[",
           extension ? "true" : "false", config ? "true" : "false", count,
           sizes ? "true" : "false", current, current_rate);
    int pairs = 0, current_has_rate = 0, current_rate_present = 0;
    if (sizes && count > 0 && count <= 4096) {
        for (int i = 0; i < count; ++i) {
            int rates_count = 0;
            short *rates = XRRConfigRates(config, i, &rates_count);
            if (i) putchar(',');
            printf("{\"index\":%d,\"width\":%d,\"height\":%d,\"rate_count\":%d,"
                   "\"rate_pointer_nonnull\":%s,\"rates\":[", i, sizes[i].width, sizes[i].height,
                   rates_count, rates ? "true" : "false");
            if (rates && rates_count > 0 && rates_count <= 4096) {
                for (int j = 0; j < rates_count; ++j) {
                    if (j) putchar(',');
                    printf("%d", rates[j]);
                    /* Xephyr advertises a real dummy 0Hz entry. Empty lists are unsafe. */
                    if (sizes[i].width > 0 && sizes[i].height > 0 && rates[j] >= 0) {
                        ++pairs;
                        if (i == current) {
                            current_has_rate = 1;
                            if (rates[j] == current_rate) current_rate_present = 1;
                        }
                    }
                }
            }
            printf("]}");
        }
    }
    int index_valid = sizes && current >= 0 && current < count && count <= 4096;
    XSync(d, False);
    int usable = !x_error && config && index_valid && pairs > 0 && current_has_rate && current_rate_present;
    printf("],\"size_rate_pair_count\":%d,\"current_index_valid\":%s,\"current_has_rate\":%s,"
           "\"current_rate_present\":%s,\"x_error\":%s,\"usable\":%s}\n", pairs,
           index_valid ? "true" : "false", current_has_rate ? "true" : "false",
           current_rate_present ? "true" : "false", x_error ? "true" : "false", usable ? "true" : "false");
    if (config) XRRFreeScreenConfigInfo(config);
    return usable ? 0 : 1;
}
static int owned_window(Display *d, const char *operation, const char *title, long pid, const char *started) {
    if (!process_matches(pid, started)) { puts("{\"owner_valid\":false}"); return 2; }
    int matches = 0, scanned = 0;
    Window found = None;
    int mutation = strcmp(operation, "window") != 0;
    if (mutation) XGrabServer(d);
    find_title(d, DefaultRootWindow(d), title, 0, &scanned, &matches, &found);
    XSync(d, False);
    long client_pid = matches == 1 ? window_pid(d, found) : -1;
    int owner_valid = !x_error && matches == 1 && client_pid == pid && process_matches(pid, started);
    XWindowAttributes before = {0}, after = {0};
    int attributes = owner_valid && XGetWindowAttributes(d, found, &before);
    int changed = 0, reparented = 0;
    if (attributes && !strcmp(operation, "map") && before.map_state != IsViewable) {
        Window root, parent, *children = NULL; unsigned int child_count = 0;
        if (XQueryTree(d, found, &root, &parent, &children, &child_count)) {
            if (parent != DefaultRootWindow(d)) {
                XReparentWindow(d, found, DefaultRootWindow(d), 0, 0); reparented = 1;
            }
        }
        if (children) XFree(children);
        XSetWindowAttributes settings = {0}; settings.override_redirect = True;
        XChangeWindowAttributes(d, found, CWOverrideRedirect, &settings);
        XMapRaised(d, found); changed = 1;
    } else if (attributes && !strcmp(operation, "unmap") && before.map_state != IsUnmapped) {
        XUnmapWindow(d, found); changed = 1;
    }
    XSync(d, False);
    int read_after = attributes && XGetWindowAttributes(d, found, &after);
    XSync(d, False);
    if (mutation) XUngrabServer(d);
    XSync(d, False);
    printf("{\"title_match_count\":%d,\"window_id\":%lu,\"xres_pid\":%ld,\"owner_valid\":%s,"
           "\"attributes_read\":%s,\"before_map_state\":%d,\"map_state\":%d,\"viewable\":%s,"
           "\"override_redirect\":%s,\"changed\":%s,\"reparented_owned_window\":%s,\"x_error\":%s}\n",
           matches, found, client_pid, owner_valid ? "true" : "false", read_after ? "true" : "false",
           before.map_state, after.map_state, read_after && after.map_state == IsViewable ? "true" : "false",
           after.override_redirect ? "true" : "false", changed ? "true" : "false",
           reparented ? "true" : "false", x_error ? "true" : "false");
    return owner_valid && read_after && !x_error ? 0 : 2;
}
int main(int argc, char **argv) {
    if (!(argc == 2 && !strcmp(argv[1], "query")) && !(argc == 5 &&
        (!strcmp(argv[1], "window") || !strcmp(argv[1], "map") || !strcmp(argv[1], "unmap")))) return 64;
    Display *d = XOpenDisplay(NULL);
    if (!d) { puts("{\"display_open\":false,\"usable\":false}"); return 1; }
    XSetErrorHandler(record_error);
    int result;
    if (argc == 2) result = query_modes(d);
    else {
        char *end = NULL; long pid = strtol(argv[3], &end, 10);
        if (!end || *end || pid <= 0) { XCloseDisplay(d); return 64; }
        result = owned_window(d, argv[1], argv[2], pid, argv[4]);
    }
    XCloseDisplay(d); return result;
}
