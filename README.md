# so101-training

SO-101 小项目工作区，用于 SO-101 机械臂的数据采集与 ACT 入门训练。所有项目文件、数据、校准记录、
模型缓存和训练输出都放在本工作区，依赖安装在独立 Conda 环境中。

GitHub 仓库包含脚本、测试代码、配置、文档与 `lerobot/` 源码快照（含本地相机示例），
保留上游许可证及依赖锁定文件，不上传数据集、缓存、模型权重或上游测试素材。
`data/`、`outputs/`、`logs/`、`.cache/`、`.tools/` 和 `.local-proxy/` 仅保留在本机，
由 `.gitignore` 排除；下文指向这些目录的链接需在本机查看。
配置和脚本中的绝对路径对应当前工作区，迁移到其他机器时需要调整，并重新准备数据、校准文件和模型。

## 参考资料

- [Seeed SO-ARM100 / SO-ARM101 入门教程](https://wiki.seeedstudio.com/cn/lerobot_so100m_new/)
- [本次使用的 LeRobot 仓库](https://github.com/Seeed-Projects/lerobot)
- [LeRobot 官方 SO-101 文档](https://huggingface.co/docs/lerobot/so101)
- [LeRobot 官方 ACT 文档](https://huggingface.co/docs/lerobot/act)
