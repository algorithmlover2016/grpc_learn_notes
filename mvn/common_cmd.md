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
mvn archetype:generate -DgroupId=com.demo -DartifactId=App

# 建一个 web 项目(linux):
mvn archetype:generate -DgroupId=com.demo -DartifactId=web-app -DarchetypeArtifactId=maven-archetype-webapp

```
[MVN in 5 minutes](https://maven.apache.org/guides/getting-started/maven-in-five-minutes.html)
```
# create a project by command line:
mvn archetype:generate -DgroupId=com.mycompany.app -DartifactId=my-app -DarchetypeArtifactId=maven-archetype-quickstart -DarchetypeVersion=1.4 -DinteractiveMode=false

# Build the Project
mvn package

# test the newly compiled and packaged JAR with the following command:

java -cp target/my-app-1.0-SNAPSHOT.jar com.mycompany.app.App
```
```
# the created pom.xml file looks like the following:
<?xml version="1.0" encoding="UTF-8"?>

<project xmlns="http://maven.apache.org/POM/4.0.0" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="http://maven.apache.org/POM/4.0.0 http://maven.apache.org/xsd/maven-4.0.0.xsd">
  <modelVersion>4.0.0</modelVersion>

  <groupId>com.mycompany.app</groupId>
  <artifactId>my-app</artifactId>
  <version>1.0-SNAPSHOT</version>

  <name>my-app</name>
  <!-- FIXME change it to the project's website -->
  <url>http://www.example.com</url>

  <properties>
    <project.build.sourceEncoding>UTF-8</project.build.sourceEncoding>
    <maven.compiler.source>1.7</maven.compiler.source>
    <maven.compiler.target>1.7</maven.compiler.target>
    <!-- set the maven.compiler.release property to the Java release you are targetting -->
    <maven.compiler.release>11</maven.compiler.release>

  </properties>

  <dependencies>
    <dependency>
      <groupId>junit</groupId>
      <artifactId>junit</artifactId>
      <version>4.11</version>
      <scope>test</scope>
    </dependency>
  </dependencies>

  <build>
    <pluginManagement><!-- lock down plugins versions to avoid using Maven defaults (may be moved to parent pom) -->
      <plugins>
        <!-- clean lifecycle, see https://maven.apache.org/ref/current/maven-core/lifecycles.html#clean_Lifecycle -->
        <plugin>
          <artifactId>maven-clean-plugin</artifactId>
          <version>3.1.0</version>
        </plugin>
        <!-- default lifecycle, jar packaging: see https://maven.apache.org/ref/current/maven-core/default-bindings.html#Plugin_bindings_for_jar_packaging -->
        <plugin>
          <artifactId>maven-resources-plugin</artifactId>
          <version>3.0.2</version>
        </plugin>
        <plugin>
          <artifactId>maven-compiler-plugin</artifactId>
          <!-- To target Java 9 (responding to maven.compiler.release property) or later, you should at least use version 3.6.0 of the maven-compiler-plugin -->
          <version>3.8.0</version>
        </plugin>
        <plugin>
          <artifactId>maven-surefire-plugin</artifactId>
          <version>2.22.1</version>
        </plugin>
        <plugin>
          <artifactId>maven-jar-plugin</artifactId>
          <version>3.0.2</version>
          <configuration>
          <archive>
            <manifest>
              <addClasspath>true</addClasspath>
              <mainClass>com.mycompany.app.App</mainClass>
            </manifest>
          </archive>
        </configuration>
        </plugin>
        <plugin>
          <artifactId>maven-install-plugin</artifactId>
          <version>2.5.2</version>
        </plugin>
        <plugin>
          <artifactId>maven-deploy-plugin</artifactId>
          <version>2.8.2</version>
        </plugin>
        <!-- site lifecycle, see https://maven.apache.org/ref/current/maven-core/lifecycles.html#site_Lifecycle -->
        <plugin>
          <artifactId>maven-site-plugin</artifactId>
          <version>3.7.1</version>
        </plugin>
        <plugin>
          <artifactId>maven-project-info-reports-plugin</artifactId>
          <version>3.0.0</version>
        </plugin>
      </plugins>
    </pluginManagement>
  </build>
</project>
```

# [Maven 依赖范围](https://www.cnblogs.com/shengulong/p/6900445.html)
* compile：编译依赖范围，使用此依赖范围对于编译、测试、运行三种classpath都有效，即在编译、测试和运行时都要使用该依赖jar包；
* test：测试依赖范围，只对测试有效，表明只在测试的时候需要，在编译和运行时将无法使用该类依赖，如 junit；
* provided：已提供依赖范围。编译和测试有效，运行无效。如servlet-api，在项目运行时，tomcat等容器已经提供，无需Maven重复引入；
* runtime：运行时依赖范围。测试和运行有效，编译无效。如 jdbc 驱动实现，编译时只需接口，测试或运行时才需要具体的 jdbc 驱动实现；
* system：系统依赖范围，使用system范围的依赖时必须通过systemPath元素显示地指定依赖文件的路径，不依赖Maven仓库解析，所以可能会造成建构的不可移植，谨慎使用。

# [Maven Phases](https://maven.apache.org/guides/getting-started/maven-in-five-minutes.html)
* **validate**: validate the project is correct and all necessary information is available
* **compile**: compile the source code of the project
* **test**: test the compiled source code using a suitable unit testing framework. These tests should not require the code be packaged or deployed
* **package**: take the compiled code and package it in its distributable format, such as a JAR.
* **integration-test**: process and deploy the package if necessary into an environment where integration tests can be run
* **verify**: run any checks to verify the package is valid and meets quality criteria
* **install**: install the package into the local repository, for use as a dependency in other projects locally
* **deploy**: done in an integration or release environment, copies the final package to the remote repository for sharing with other developers and projects.

* **clean**: cleans up artifacts created by prior builds
    * eg `mvn clean dependency:copy-dependencies package`
* **site**: generates site documentation for this project
    * eg `mvn site`
