FROM althack/ros2:humble-dev 

# ** [Optional] Uncomment this section to install additional packages. **
# 设置环境变量避免交互式界面
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends vim \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
# 重置 DEBIAN_FRONTEND 环境变量
ENV DEBIAN_FRONTEND=dialog

# 定义构建参数 WORKSPACE
ARG WORKSPACE
# 在 /home/ros/.bashrc 文件（通常为用户的 shell 启动脚本）的末尾添加了一行代码。
# 添加的代码是一个条件语句，用于检查在 WORKSPACE 路径下的 install 子目录中是否存在 setup.bash 文件。如果该文件存在，该语句将执行 source 命令来运行 setup.bash 脚本，这通常用于设置环境变量或执行其他初始化操作。
#  使用 >> 重定向操作符确保新内容被追加到 .bashrc 文件的末尾，而不是覆盖现有内容。
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
