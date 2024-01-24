# 如何贡献文档

相关内容请参考[https://rflybuaa.github.io/RflySimRTDoc/help/readme/](https://rflybuaa.github.io/RflySimRTDoc/help/readme/)

# 如何生成PDF

相关内容请参考[https://pypi.org/project/mkdocs-with-pdf/](https://pypi.org/project/mkdocs-with-pdf/)。
!!! TIP
    使用``mkdocs-with-pdf``插件需要提前安装weasyprint。在安装weasyprint之前，需要先安装[GTK3.x](https://github.com/tschoonj/GTK-for-Windows-Runtime-Environment-Installer/releases)。在安装GTK3.x时，推荐使用默认的安装选项，安装完成后重启（使环境变量生效）。
    安装weasyprint: ``python -m pip install weasyprint``

启用pdf生成，修改``mkdocs.yml``。
```
plugins:
  - search
  - autorefs
  # - git-latest-changes #展示最近一次更新的变化
  # - git-latest-release
  - mkdocs-video:
      is_video: true
      video_autoplay: false
  - resize-images:
      source-dir: img-large
      target-dir: img
      enable_cache: True
      size: [400, 400]
      extensions: ['.jpg', '.jpeg', '.png', '.svg']
  - with-pdf:
        verbose: true
```
然后执行命令``mkdocs build``，随后将在``site/pdf``文件夹中生成PDF文件。

!!! TIP
    建议在生成pdf后，取消改插件设置。