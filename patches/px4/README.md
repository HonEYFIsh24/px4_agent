# 本机 PX4 修改的源码快照

此目录保存工程开发机上现有 PX4 工作树相对于上游提交 `b8c541dd7277ed735139d7d1bfb829d61fbe29fb` 的四个已跟踪文件差异，避免仅上传 Agent 而遗漏实际 SITL／CLI 的修改。它不是本次上传任务新增的飞控改动。

- 上游：https://github.com/PX4/PX4-Autopilot
- 版本基点、补丁 SHA256 和当前源文件 SHA256 见 `source_manifest.json`。
- 上游许可证原样保存为 `PX4-LICENSE`；补丁中的既有版权声明保留。

在与基点匹配且无冲突的 PX4 checkout 中先执行 `git apply --check /path/to/px4-agent/patches/px4/local-sitl-and-cli.patch`，再执行 `git apply` 并按 PX4 的依赖要求重建 `px4_sitl_default`。不要对已有本机修改重复应用。仿真脚本可能还需按自己的工作目录设置 `PX4_DIR`；具体安装约束见 `docs/GITHUB_SETUP.md`。

预编译 PX4、构建缓存及历史实验日志不包含在此仓库。源文件一致不等于新机器编译结果或飞行验收通过；需独立构建和验证。
