# [the detail explanation of pom.xml](https://maven.apache.org/pom.html)
## [配置文件概览](http://www.shixinke.com/java/maven-configuration-file-pom-xml-introduction)
```
<project xmlns="http://maven.apache.org/POM/4.0.0"
    xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:schemaLocation="http://maven.apache.org/POM/4.0.0 http://maven.apache.org/xsd/maven-4.0.0.xsd">
    <!--模型的版本，即pom的版本，maven2或maven3对应的是4.0.0 -->
    <modelVersion>4.0.0</modelVersion>

    <!-- 基础设置 -->
    <!-- 公司或者组织的唯一标志，并且配置时生成的路径也是由此生成， 如com.shixinke.order，maven会将该项目打成的jar包放本地路径：/com/shixinke/order -->
    <groupId>com.shixinke.order</groupId>

    <!-- 本项目的唯一ID，一个groupId下面可能多个项目，就是靠artifactId来区分的 -->
    <artifactId>shixinke-order</artifactId>

    <!-- 本项目当前所处版本 -->
    <version>1.0.0</version>

    <!-- 打包的机制，如pom,jar, maven-plugin, ejb, war, ear, rar, par，默认为jar -->
    <packaging>war</packaging>

    <!-- 帮助定义构件输出的一些附属构件,附属构件与主构件对应，有时候需要加上classifier才能唯一的确定该构件 不能直接定义项目的classifer,因为附属构件不是项目直接默认生成的，而是由附加的插件帮助生成的 -->
    <classifier>...</classifier>

    <!-- 定义本项目的依赖关系 -->
    <dependencies>
        <!-- 每个dependency都对应这一个jar包 -->
        <dependency>
        <!--一般情况下，maven是通过groupId、artifactId、version这三个元素值（俗称坐标）来检索该构件， 然后引入你的工程。如果别人想引用你现在开发的这个项目（前提是已开发完毕并发布到了远程仓库），-->
        <!--就需要在他的pom文件中新建一个dependency节点，将本项目的groupId、artifactId、version写入， maven就会把你上传的jar包下载到他的本地 -->
        <groupId>com.winner.trade</groupId>
        <artifactId>trade-test</artifactId>
        <version>1.0.0-SNAPSHOT</version>
        <!-- maven认为，程序对外部的依赖会随着程序的所处阶段和应用场景而变化，所以maven中的依赖关系有作用域(scope)的限制。 -->
        <!--scope包含如下的取值：compile（编译范围）、provided（已提供范围）、runtime（运行时范围）、test（测试范围）、system（系统范围） -->
        <scope>test</scope>
        <!-- 设置指依赖是否可选，默认为false,即子项目默认都继承:为true,则子项目必需显示的引入，与dependencyManagement里定义的依赖类似  -->
        <optional>false</optional>
        <!-- 屏蔽依赖关系。 比如项目中使用的libA依赖某个库的1.0版，libB依赖某个库的2.0版，现在想统一使用2.0版，就应该屏蔽掉对1.0版的依赖 -->
        <exclusions>
        <exclusion>
        <groupId>org.slf4j</groupId>
        <artifactId>slf4j-api</artifactId>
        </exclusion>
        </exclusions>
        </dependency>
    </dependencies>
    <!-- 定义本pom的父级模型，可以从父级模型中继承其相关的属性 -->
    <parent>...</parent>
    <parent>
   <groupId>com.project.main</groupId>
   <artifactId>project-module</artifactId>
   <version>1.0.2</version>
   <relativePath>../pom.xml</relativePath>  <!--本例中此处是可选的-->
</parent>
    <!-- 依赖管理。 dependencyManagement 中的 dependencies 元素只表明依赖项版本的优先选择，并不影响项目的依赖项；而 dependencies 元素则影响项目的依赖项。 -->
    <dependencyManagement>...</dependencyManagement>
    <!--多模块定义-->
    <modules>...</modules>
    <!-- 为pom定义一些常量，在pom中的其它地方可以直接引用 使用方式 如下 ：${file.encoding} -->
    <properties>...</properties>
    <!-- 构建设置 -->
    <!-- 项目构建配置，包括构建目录、插件等 -->
    <build>...</build>
    <!--用于生成报表。<reporting>中也可以配置插件<plugins>，并通过一个<plugin>的<reportSet>为该插件配置参数-->
    <reporting>...</reporting>
    <!--项目的名称, Maven产生的文档用 -->
    <!--项目名称-->
    <name>...</name>
    <!--项目描述-->
    <description>...</description>
    <!--项目主页的URL, Maven产生的文档用 -->
    <url>...</url>
    <!--项目创建年份，4位数字。当产生版权信息时需要使用这个值。 -->
    <inceptionYear>...</inceptionYear>
    <!--该元素描述了项目所有License列表。应该只列出该项目的license列表，不要列出依赖项目的license列表。 -->
    <!--如果列出多个license，用户可以选择它们中的一个而不是接受所有license。 -->
    <licenses>...</licenses>
    <!--项目开发者所属组织 -->
    <organization>...</organization>
    <!--项目开发者列表 -->
    <developers>...</developers>
    <!--项目贡献者列表 -->
    <contributors>...</contributors>
    <!-- 环境设置 -->
    <!--项目的问题管理系统(Bugzilla, Jira, Scarab,或任何你喜欢的问题管理系统)的名称和URL -->
    <issueManagement>...</issueManagement>
    <!--项目持续集成信息 -->
    <ciManagement>...</ciManagement>
    <!--项目相关邮件列表信息 -->
    <mailingLists>...</mailingLists>
    <!--SCM(Source Control Management)标签允许你配置你的代码库，供Maven web站点和其它插件使用。 -->
    <scm>...</scm>
    <!--描述了这个项目构建环境中的前提条件。 -->
    <prerequisites>...</prerequisites>
    <!--发现依赖和扩展的远程仓库列表。 -->
    <repositories>...</repositories>
    <!--发现插件的远程仓库列表，这些插件用于构建和报表 -->
    <pluginRepositories>...</pluginRepositories>
    <!--项目分发信息，在执行mvn deploy后表示要发布的位置。 -->
    <!--有了这些信息就可以把网站部署到远程服务器或者把构件部署到远程仓库。 -->
    <distributionManagement>...</distributionManagement>
    <!--在列的项目构建profile，如果被激活，会修改构建处理 -->
    <profiles>...</profiles>
    </project>
```

# [Maven Properties Guide](https://cwiki.apache.org/confluence/display/MAVEN/Maven+Properties+Guide)
* ***`${project.basedir}`***
    * This references to the root folder of the module/project (the location where the current `pom.xml` file is located)
    * POM properties referencing useful build locations, with default values defined in the **[Super POM](http://maven.apache.org/pom.html#The_Super_POM)**:

* ***`${project.build.directory}`***
    * This represents by default the **`target`** folder.
* ***`${project.build.outputDirectory}`***
    * This represents by default the **`target/classes`** folder.
* ***`${project.build.testOutputDirectory}`***
    * This represents by default the **`target/test-classes`** folder.
* ***`${project.build.sourceDirectory}`***
    * This represents by default the **`src/main/java`** folder.
* ***`${project.build.testSourceDirectory}`***
    * This represents by default the **`src/test/java`** folder.

* ***`${project.build.finalName}`***
    * This is by default defined as **`${project.artifactId}-${project.version}`**
* ***`${project.version}`***
    * This can be used at locations where you have to write a literal version otherwise, in particular if you are in a multi-modules build for inter modules dependencies.


* ***`${settings.localRepository}`***
    * The settings.xml elements could be referenced by using things like this (see also at the Super POM):
        which references the location of the local repository. This is by default **`${home}/.m2/repository`**.
## references:
```
https://maven.apache.org/ref/3.8.3/maven-model-builder/#Model_Interpolation
https://maven.apache.org/ref/3.8.3/maven-model/maven.html
```
