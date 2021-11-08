# command command:
[常用命令](https://blog.csdn.net/superit401/article/details/51355241)
```
# 创建 Maven 项目
mvn archetype:create

# 编译源代码
mvn compile

# 编译测试代码
mvn test-compile

# 运行应用程序中的单元测试
mvn test

# 生成项目相关信息的网站
mvn site

# 清除目标目录中的生成结果
mvn clean

# 依据项目生成 jar 文件
mvn package

# 在本地 Repository 中安装 jar
mvn install

# 生成 Eclipse 项目文件
mvn eclipse:eclipse

# 生成站点目录并发布
mvn site-deploy
```
```
# examples:生成项目
# 建一个 JAVA 项目:
mvn archetype:create -DgroupId=com.demo -DartifactId=App

# 建一个 web 项目:
mvn archetype:create -DgroupId=com.demo -DartifactId=web-app -DarchetypeArtifactId=maven-archetype-webapp
```
