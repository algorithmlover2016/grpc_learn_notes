# reference to https://www.jianshu.com/p/29cca79d343c
colcon build --symlink-install --cmake-force-configure
colcon list
colcon build --packages-select <name-of-pkg>
colcon build --packages-up-to <name-of-pkg>
COLCON_IGNORE # 如果不希望编译某一个 package，可以在该 package中创建名为 COLCON_IGNORE 的空文件，colcon 就会忽略掉该 package
