# 可选 Gazebo Classic 物理同步后端

本目录保留针对 Gazebo Classic **11.15.1** 的源代码补丁、上游许可和原开发机 manifest。动态库与构建目录不上传。默认 `gazebo_sync:=false` 使用系统 Gazebo；只有完成对应机器的独立构建、校验和仿真验收后，才能考虑开启私有后端。

补丁修改 `gazebo/physics/MultiRayShape.cc` 和 `gazebo/physics/World.cc`，将多射线更新与世界物理、碰撞位姿更新放在同一物理更新锁的保护范围，插件回调在释放锁后调用。它不构成对所有外部插件、物理引擎或动态障碍场景的完整正确性保证，也不能据此给历史短距返回或进程崩溃归因。

## 来源与可复核文件

上游仓库为 [gazebosim/gazebo-classic](https://github.com/gazebosim/gazebo-classic)，tag 为 `gazebo11_11.15.1`。原构建使用的上游源码 archive SHA-256：

```text
77a952971eabfd79e16d2582a355ee017a2e22cc369b68453323c0984b725126
```

以下文件逐字复制自原开发机保存的补丁和上游源码，未改写许可文本：

| 文件 | 字节数 | SHA-256 |
|---|---:|---|
| `patches/atomic_ray_update_sync.patch` | 2347 | `50b7c8db8a6a4de6b26f56d5f57c3a2be78b5231db879252d2f6dc3a887cd3b8` |
| `licenses/Gazebo-LICENSE` | 621 | `d26974a952e546ec998cc18efa7a043b33d5edc06d3019ba1bad453955051968` |
| `licenses/Gazebo-COPYING` | 10175 | `e1925845017bf307c7d465af432e8bb20cbb6ce17b10cd323af59da717753075` |

`Gazebo-LICENSE` 包含 Open Source Robotics Foundation 的 Apache 2.0 声明，`Gazebo-COPYING` 保存许可全文。

## 原构建方式

以下是历史构建参数按目录变量表达的示例；它们不是本仓库提供的自动安装器。先在独立工作目录准备版本和 SHA 已核对的上游源码，并确保依赖与目标系统 Gazebo 兼容：

```bash
export GAZEBO_SYNC_SOURCE=/absolute/path/to/gazebo11_11.15.1-source
export GAZEBO_SYNC_BUILD=/absolute/path/to/private-gazebo-build
export GAZEBO_SYNC_PREFIX=/absolute/path/to/private-gazebo-prefix
patch --batch -p1 -d "$GAZEBO_SYNC_SOURCE" \
  -i "$PWD/vio/gazebo_sync/patches/atomic_ray_update_sync.patch"
cmake -S "$GAZEBO_SYNC_SOURCE" -B "$GAZEBO_SYNC_BUILD" \
  -DBUILD_TESTING=OFF -DUSE_PCH=OFF \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_INSTALL_PREFIX="$GAZEBO_SYNC_PREFIX" \
  -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build "$GAZEBO_SYNC_BUILD" --target gazebo_physics --parallel 2
```

原构建记录为本机 `logs/ev/atomic_ray_update_sync_design_01/source_patch_01/private_build_01/build_result.json`；该运行日志不随源码上传。原最终库经过独立的 ELF 清理、依赖加载和 ABI 检查，上述 `cmake` 命令本身不保证产生相同字节的最终库。不要执行全局安装或覆盖 `/usr/lib` 中的系统 Gazebo。

## manifest 与新机器的边界

现有 [manifest.json](manifest.json) 是原开发机冻结证据，包含 `/home/dut/...` 绝对路径、原系统依赖和库哈希。原库 SHA-256 为 `f305b780960b8751a0e5980e525fd0ce0beabe6e362aaef639e28b84877a940f`，但本仓库没有携带该 `.so`。manifest 不是可移植的包清单，也不表示新机器已经通过运行验收。

`scripts/validate_gazebo_sync.py` 会核对库、manifest 和依赖。直接启用 `gazebo_sync:=true` 而缺少原库，或只修改路径／哈希使检查通过，都不构成后端移植。新机器需要重新确认系统依赖、SONAME 链接、无构建目录 RPATH/RUNPATH、公开导出 ABI 和实际装载结果，形成与新构建匹配的可审查记录，再进行完整飞行及故障测试。

原设计仅向本次 `gzserver` 注入私有物理库路径；`gzclient` 保留系统 Gazebo 依赖。不要把私有库通过全局 `LD_LIBRARY_PATH` 注入所有进程。使用系统库时也需要重新验收，不能继承历史私有后端的性能或安全测试结论。
