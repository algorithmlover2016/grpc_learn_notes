# command command:
## [常用命令](https://blog.csdn.net/superit401/article/details/51355241)
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
## [MVN in 5 minutes](https://maven.apache.org/guides/getting-started/maven-in-five-minutes.html)
```
# create a project by command line:
mvn archetype:generate -DgroupId=com.mycompany.app -DartifactId=my-app -DarchetypeArtifactId=maven-archetype-quickstart -DarchetypeVersion=1.4 -DinteractiveMode=false

# Build the Project
mvn package

# test the newly compiled and packaged JAR with the following command:
java -cp target/my-app-1.0-SNAPSHOT.jar com.mycompany.app.App
```
the created ***`pom.xml`*** file looks like the following:
```
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
                <!-- NOTE: We don't need a groupId specification because the group is
                            org.apache.maven.plugins ...which is assumed by default.
                -->
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
* **compile**：编译依赖范围，使用此依赖范围对于编译、测试、运行三种classpath都有效，即在编译、测试和运行时都要使用该依赖jar包；
* **test**：测试依赖范围，只对测试有效，表明只在测试的时候需要，在编译和运行时将无法使用该类依赖，如 junit；
* **provided**：已提供依赖范围。编译和测试有效，运行无效。如servlet-api，在项目运行时，tomcat等容器已经提供，无需Maven重复引入；
* **runtime**：运行时依赖范围。测试和运行有效，编译无效。如 jdbc 驱动实现，编译时只需接口，测试或运行时才需要具体的 jdbc 驱动实现；
* **system**：系统依赖范围，使用system范围的依赖时必须通过systemPath元素显示地指定依赖文件的路径，不依赖Maven仓库解析，所以可能会造成建构的不可移植，谨慎使用。

# [Maven Phases](https://maven.apache.org/guides/getting-started/maven-in-five-minutes.html)
* **validate**: validate the project is correct and all necessary information is available
* **compile**: compile the source code of the project
* **test**: test the compiled source code using a suitable unit testing framework. These tests should not require the code be packaged or deployed
* **package**: take the compiled code and package it in its distributable format, such as a JAR.
* **integration-test**: process and deploy the package if necessary into an environment where integration tests can be run
* **verify**: run any checks to verify the package is valid and meets quality criteria
* **install**: install the package into the local repository, for use as a dependency in other projects locally
* **deploy**: done in an integration or release environment, copies the final package to the remote repository for sharing with other developers and projects.
<\b>
* **clean**: cleans up artifacts created by prior builds
    * eg `mvn clean dependency:copy-dependencies package`
* **site**: generates site documentation for this project
    * eg `mvn site`
# [Maven package](https://blog.csdn.net/sgyuanshi/article/details/98115890)
## 只打包自己project的source code, 不打包任何依赖包, 依赖包跟项目代码一同编译
```
<?xml version="1.0" encoding="UTF-8"?>
<project xmlns="http://maven.apache.org/POM/4.0.0"
         xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
         xsi:schemaLocation="http://maven.apache.org/POM/4.0.0 http://maven.apache.org/xsd/maven-4.0.0.xsd">
    <modelVersion>4.0.0</modelVersion>
    ...
    <properties>
        <maven.compiler.source>1.8</maven.compiler.source>
        <maven.compiler.target>1.8</maven.compiler.target>
        <project.build.sourceEncoding>UTF-8</project.build.sourceEncoding>
        <project.reporting.outputEncoding>UTF-8</project.reporting.outputEncoding>
        <maven.compiler.plugin>3.8.1</maven.compiler.plugin>
        <maven.assembly.plugin>3.3.0</maven.assembly.plugin>
    </properties>
    ...
    <build>
        ...
        <plugins>
            <!-- only build source codes in src/main not including dependencies into jar files, default plugin-->
            <plugin>
                <groupId>org.apache.maven.plugins</groupId>
                <artifactId>maven-jar-plugin</artifactId>
                <configuration>
                    <archive>
                        <manifest>
                            <addClasspath>true</addClasspath>
                            <mainClass>DuomaiApi</mainClass>
                        </manifest>
                    </archive>
                </configuration>
            </plugin>
            <plugin>
                <groupId>org.apache.maven.plugins</groupId>
                <artifactId>maven-compiler-plugin</artifactId>
                <version>${maven.compiler.plugin}</version>
                <configuration>
                    <source>${maven.compiler.source}</source>
                    <target>${maven.compiler.target}</target>
                    <encoding>${project.build.sourceEncoding}</encoding>
                </configuration>
            </plugin>
            ...
        </plugins>
        ...
    </build>
</project>
```

## 只打包仓库依赖包，不打包本地依赖包(scope为system的依赖不打包), 依赖包跟项目代码一同编译
```
<project>
    ...
    <properties>
        <maven.compiler.source>1.8</maven.compiler.source>
        <maven.compiler.target>1.8</maven.compiler.target>
        <project.build.sourceEncoding>UTF-8</project.build.sourceEncoding>
        <project.reporting.outputEncoding>UTF-8</project.reporting.outputEncoding>
        <maven.compiler.plugin>3.8.1</maven.compiler.plugin>
        <maven.assembly.plugin>3.3.0</maven.assembly.plugin>
    </properties>
    ...
    <build>
        <plugins>
            <plugin>
                <groupId>org.apache.maven.plugins</groupId>
                <artifactId>maven-compiler-plugin</artifactId>
                <version>${maven.compiler.plugin}</version>

                <configuration>
                    <source>${maven.compiler.source}</source>
                    <target>${maven.compiler.target}</target>
                    <encoding>${project.build.sourceEncoding}</encoding>
                </configuration>
            </plugin>
            <plugin>
                <groupId>org.apache.maven.plugins</groupId>
                <artifactId>maven-assembly-plugin</artifactId>
                <version>${maven.assembly.plugin}</version>

                <configuration>
                    <archive>
                        <manifest>
                            <mainClass>your.main.class</mainClass>
                        </manifest>
                        <!--
                        <manifestEntries>
                            <Class-Path>. </Class-Path>
                        </manifestEntries>
                        -->
                    </archive>
                    <descriptorRefs>
                        <descriptorRef>jar-with-dependencies<descriptorRef>
                    </descriptorRefs>
                </configuration>
                <executions>
                    <execution>
                        <id>make-assembly</id>
                        <phase>package</phase>
                        <goals>
                            <goal>single</goal>
                        </goals>
                    </execution>
                </executions>
            </plugin>
            ...
        </plugins>
    </build>
    ...
</project>
```

## 打包仓库依赖包和本地依赖包(scope为system的依赖), 依赖包跟项目代码一同编译
* **firstly**, create a local file named ***`assembly.xml`*** under your project, for example in ***`src/main/assembly.xml`*** and its content is as follows:
```
<?xml version="1.0" encoding="UTF-8"?>
<assembly>
    <id>jar-with-dependencies</id>
    <formats>
        <format>jar</format>
    </formats>
    <includeBaseDirectory>false</includeBaseDirectory>
    <dependencySets>
        <!-- default configuration -->
        <dependencySet>
            <outputDirectory>/</outputDirectory>
            <useProjectArtifact>true</useProjectArtifact>
            <unpack>true</unpack>
            <scope>runtime</scope>
        </dependencySet>

        <!-- specify the configuration whose scope is system-->
        <dependencySet>
            <outputDirectory>/</outputDirectory>
            <useProjectArtifact>true</useProjectArtifact>
            <unpack>true</unpack>
            <scope>system</scope>
        </dependencySet>
    </dependencySets>
</assembly>
```
* **secondly**, modify your pom.xml at some places to make ***`assembly.xml`*** effect.
```
<?xml version="1.0" encoding="UTF-8"?>
<project xmlns="http://maven.apache.org/POM/4.0.0"
         xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
         xsi:schemaLocation="http://maven.apache.org/POM/4.0.0 http://maven.apache.org/xsd/maven-4.0.0.xsd">
    <modelVersion>4.0.0</modelVersion>
    ...
    <properties>
        <maven.compiler.source>1.8</maven.compiler.source>
        <maven.compiler.target>1.8</maven.compiler.target>
        <project.build.sourceEncoding>UTF-8</project.build.sourceEncoding>
        <project.reporting.outputEncoding>UTF-8</project.reporting.outputEncoding>
        <maven.compiler.plugin>3.8.1</maven.compiler.plugin>
        <maven.assembly.plugin>3.3.0</maven.assembly.plugin>
    </properties>
    ...
    <build>
        ...
        <plugins>
            ...
            <plugin>
                <groupId>org.apache.maven.plugins</groupId>
                <artifactId>maven-compiler-plugin</artifactId>
                <version>${maven.compiler.plugin}</version>
                <configuration>
                    <source>${maven.compiler.source}</source>
                    <target>${maven.compiler.target}</target>
                    <encoding>${project.build.sourceEncoding}</encoding>
                </configuration>
            </plugin>
            <plugin>
                <groupId>org.apache.maven.plugins</groupId>
                <artifactId>maven-assembly-plugin</artifactId>
                <version>${maven.assembly.plugin}</version>

                <configuration>
                    <archive>
                        <manifest>
                            <mainClass>your.main.class</mainClass>
                        </manifest>
                    </archive>
                </configuration>
                <executions>
                    <execution>
                        <id>make-assembly</id>
                        <phase>package</phase>
                        <goals>
                            <goal>single</goal>
                        </goals>
                        <configuration>
                            <descriptors>
                                <descriptor>assembly.xml</descriptor>
                            </descriptors>
                        </configuration>
                    </execution>
                </executions>
            </plugin>
            ...
        </plugins>
        ...
    </build>
</project>
```

## 不编译依赖包，只打包到指定目录
* ***增加了一个`maven-dependency-plugin`插件***

```
<?xml version="1.0" encoding="UTF-8"?>
<project xmlns="http://maven.apache.org/POM/4.0.0"
         xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
         xsi:schemaLocation="http://maven.apache.org/POM/4.0.0 http://maven.apache.org/xsd/maven-4.0.0.xsd">
    <modelVersion>4.0.0</modelVersion>
    ...
    <properties>
        <maven.compiler.source>1.8</maven.compiler.source>
        <maven.compiler.target>1.8</maven.compiler.target>
        <project.build.sourceEncoding>UTF-8</project.build.sourceEncoding>
        <project.reporting.outputEncoding>UTF-8</project.reporting.outputEncoding>
        <maven.compiler.plugin>3.8.1</maven.compiler.plugin>
        <maven.assembly.plugin>3.3.0</maven.assembly.plugin>
    </properties>
    ...
    <build>
        ...
        <plugins>
            ...
            <plugin>
                <groupId>org.apache.maven.plugins</groupId>
                <artifactId>maven-dependency-plugin</artifactId>
                <version>${maven.dependency.plugin}</version>
                <executions>
                    <execution>
                        <id>copy-dependencies</id>
                        <phase>package</phase>
                        <goals>
                            <goal>copy-dependencies</goal>
                        </goals>
                        <configuration>
                            <outputDirectory>${project.build.directory}/libs</outputDirectory>
                            <overWriteRelease>false</overWriteRelease>
                            <overWriteSnapshots>false</overWriteSnapshots>
                            <overWriteIfNewer>true</overWriteIfNewer>
                        </configuration>
                    </execution>
                </executions>
            </plugin>
            <plugin>
                <groupId>org.apache.maven.plugins</groupId>
                <artifactId>maven-compiler-plugin</artifactId>
                <version>${maven.compiler.plugin}</version>
                <configuration>
                    <source>${maven.compiler.source}</source>
                    <target>${maven.compiler.target}</target>
                    <encoding>${project.build.sourceEncoding}</encoding>
                </configuration>
            </plugin>
            <plugin>
                <groupId>org.apache.maven.plugins</groupId>
                <artifactId>maven-assembly-plugin</artifactId>
                <version>${maven.assembly.plugin}</version>

                <configuration>
                    <archive>
                        <manifest>
                            <mainClass>your.main.class</mainClass>
                        </manifest>
                    </archive>
                </configuration>
                <executions>
                    <execution>
                        <id>make-assembly</id>
                        <phase>package</phase>
                        <goals>
                            <goal>single</goal>
                        </goals>
                        <configuration>
                            <descriptors>
                                <descriptor>assembly.xml</descriptor>
                            </descriptors>
                        </configuration>
                    </execution>
                </executions>
            </plugin>
            ...
        </plugins>
        ...
    </build>
</project>
```
# [常见的maven插件](https://segmentfault.com/a/1190000016237395)
* ***`maven-jar-plugin`***        maven 默认打包插件，用来创建 project jar
* ***`maven-shade-plugin`***      用来打可执行包，executable(fat) jar
* ***`maven-assembly-plugin`***   支持定制化打包方式，例如 apache 项目的打包方式
## maven-compiler-plugin
configure looks like this when using **`maven-compiler-plugin`**
```
<plugin>
    <groupId>org.apache.maven.plugins</groupId>
    <artifactId>maven-compiler-plugin</artifactId>
    <version>3.8.1</version>
    <configuration>
        <source>1.8</source>
        <target>1.8</target>
    </configuration>
</plugin>
```
or
```
<properties>
        <project.build.sourceEncoding>UTF-8</project.build.sourceEncoding>
        <maven.compiler.source>1.8</maven.compiler.source>
        <maven.compiler.target>1.8</maven.compiler.target>
</properties>
```

## maven-jar-plugin
configure looks like as follows:
```
<plugin>
    <groupId>org.apache.maven.plugins</groupId>
    <artifactId>maven-jar-plugin</artifactId>
    <version>2.4</version>
    <configuration>
        <archive>
            <manifest>
                <addClasspath>true</addClasspath>
                <classpathPrefix>/data/lib</classpathPrefix>
                <mainClass>com.zhang.spring.App</mainClass>
            </manifest>
        </archive>
    </configuration>
</plugin>
```
## maven-assembly-plugin
### pom.xml中配置maven的assembly插件
```
<build>
    <plugins>
        <plugin>
            <groupId>org.apache.maven.plugins</groupId>
            <artifactId>maven-assembly-plugin</artifactId>
            <configuration>
                <archive>
                    <manifest>
                        <mainClass>your.packge_name.main_class_name</mainClass>
                    </manifest>
                </archive>
            </configuration>
            <executions>
                <execution>
                    <!-- 配置执行器 -->
                    <id>make-assembly</id>
                    <phase>package</phase><!-- 绑定到package生命周期阶段上 -->
                    <!--  (1) -->
                    <goals>
                        <goal>single</goal><!-- 只运行一次 -->
                    </goals>

                    <!-- (2) -->
                    <configuration>
                        <!-- Define the name of the output jar -->
                        <finalName>${project.name}</finalName>

                        <!-- Do not append descriptor id (assembly_id) to $finalName, default is true -->
                        <appendAssemblyId>false</appendAssemblyId>

                        <!-- Specify the output directory -->
                        <outputDirectory>${project.basedir}/out</outputDirectory>

                        <descriptors>
                            <!--配置描述文件路径-->
                            <descriptor>src/main/assembly/assembly.xml</descriptor>
                        </descriptors>
                    </configuration>
                </execution>
            </executions>
          </plugin>
        </plugins>
</build>
```
#### 内置的Assembly Descriptor
* ***`bin`***                         类似于默认打包，会将bin目录下的文件打到包中
* ***`jar-with-dependencies`***       会将所有依赖都解压打包到生成物中
* ***`src`***                         只将源码目录下的文件打包
* ***`project`***                     将整个project资源打包
<\b>
##### 使用 descriptorRefs来引用(官方提供的定制化打包方式)
```
<plugin>
    <artifactId>maven-assembly-plugin</artifactId>
    ...
    <configuration>
        ...
        <descriptorRefs>
            <descriptorRef>jar-with-dependencies</descriptorRef>
        </descriptorRefs>
    </configuration>
</plugin>
```
#### ***[user-defined assembly desciptor](http://maven.apache.org/plugins/maven-assembly-plugin/assembly.html)***
##### **[content in assembly.xml](https://www.jianshu.com/p/14bcb17b99e0)**
* ***`id`***
* ***`formats`***
* ***`dependencySets`***
    * **`outputDirectory`**   **`String`**          指定包依赖目录，该目录是相对于根目录
    * **`includes/include*`** **`List<String>`**    包含依赖
    * **`excludes/exclude*`** **`List<String>`**    排除依赖
```
<dependencySets>
    <dependencySet>
        <outputDirectory>/lib</outputDirectory>
    </dependencySet>
</dependencySets>
```
* ***`fileSets`***
    * **`outputDirectory`**     **`String`**          指定文件集合的输出目录，该目录是相对于根目录
    * **`includes/include*`**	**`List<String>`**    包含文件
    * **`excludes/exclude*`**	**`List<String>`**    排除文件
    * **`fileMode`**            **`String`**          指定文件属性，使用八进制表达，分别为(User)(Group)(Other)所属属性，默认为 0644
```
<fileSets>
    <fileSet>
        <includes>
            <include>bin/**</include>
        </includes>
        <fileMode>0755</fileMode>
    </fileSet>

    <fileSet>
        <includes>
            <include>/conf/**</include>
            <include>logs</include>
        </includes>
    </fileSet>

</fileSets>
```
* ***`files`***
    * **`source`**           **`String`**    源文件，相对路径或绝对路径
    * **`outputDirectory`**  **`String`**    输出目录
    * **`destName`**         **`String`**    目标文件名
    * **`fileMode`**         **`String`**    设置文件 UNIX 属性
```
<files>
    <file>
        <source>README.txt</source>
        <outputDirectory>/</outputDirectory>
    </file>
</files>
```
##### ***`bin`*** template
```
<assembly xmlns="http://maven.apache.org/plugins/maven-assembly-plugin/assembly/1.1.0"
    xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:schemaLocation="http://maven.apache.org/plugins/maven-assembly-plugin/assembly/1.1.0 http://maven.apache.org/xsd/assembly-1.1.0.xsd">
    <id>bin</id>
    <formats>
        <format>tar.gz</format>
        <format>tar.bz2</format>
        <format>zip</format>
    </formats>
    <fileSets>
        <fileSet>
            <directory>${project.basedir}</directory>
            <outputDirectory>/</outputDirectory>
            <includes>
                <include>README*</include>
                <include>LICENSE*</include>
                <include>NOTICE*</include>
            </includes>
        </fileSet>
        <fileSet>
            <directory>${project.build.directory}</directory>
            <outputDirectory>/</outputDirectory>
            <includes>
                <include>*.jar</include>
            </includes>
        </fileSet>
        <fileSet>
            <directory>${project.build.directory}/site</directory>
            <outputDirectory>docs</outputDirectory>
        </fileSet>
    </fileSets>
</assembly>
```
##### ***[assembly configuration format](https://cloud.tencent.com/developer/article/1622206)*** assuming that it's located in **`src/main/assembly/assembly.xml`**
```
<assembly xmlns="http://maven.apache.org/ASSEMBLY/2.0.0"
          xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
          xsi:schemaLocation="http://maven.apache.org/ASSEMBLY/2.0.0 http://maven.apache.org/xsd/assembly-2.0.0.xsd
http://maven.apache.org/ASSEMBLY/2.0.0 ">

    <!--唯一ID-->
    <id>assembly_test</id>

    <!--打包格式，允许同时有多个-->
    <formats>
        <format>tar.gz</format>
        <format>dir</format>
        <format>zip</format>
    </formats>

    <!--依赖jar包以及项目打包文件存储文件-->
    <dependencySets>
        <dependencySet>
            <!--存储在projectName-assembly-version/lib下-->
            <outputDirectory>lib</outputDirectory>
        </dependencySet>
    </dependencySets>

    <fileSets>
        <fileSet>
            <!--目录路径，如果不在这里指定，而在include中指定，那么其文件夹的也会被带进去-->
            <directory>src/main/bin/</directory>
            <includes>
                <!--要哪些文件-->
                <include>*.*</include>
            </includes>
            <excludes>
                <!--不要哪些文件-->
                <exclude>*.no_need</exclude>
            </excludes>
            <!--文件的权限-->
            <fileMode>0755</fileMode>
            <!--输出目录 存储在projectName-assembly-version/bin下-->
            <outputDirectory>bin</outputDirectory>
            <directoryMode>0755</directoryMode>

        </fileSet>
    </fileSets>

    <files>
        <!--针对单个文件-->
        <file>
            <!--源文件地址，相对于项目地址-->
            <source>pom.xml</source>
            <!--输出目录为projectName-assembly-version/-->
            <outputDirectory>.</outputDirectory>
            <!--文件的权限-->
            <fileMode>0755</fileMode>
            <!--重命名为-->
            <destName>pom.xml</destName>
        </file>
    </files>
</assembly>
```
