# 简易的命令行入门教程:
Git 全局设置:
```bash
git config --global user.name "TonyKerman"
git config --global user.email "2676239430@qq.com"
```
创建 git 仓库:
```bash
mkdir lib
cd lib
git init 
touch README.md
git add README.md
git commit -m "first commit"
git remote add origin git@gitee.com:TonyKerman/[repo].git
git push -u origin "master"
```
已有仓库?
```bash
cd existing_git_repo
git remote add origin git@gitee.com:TonyKerman/lib.git
git push -u origin "master"
```
# git submodule
git submodule 命令用于管理包含其他 Git 仓库的项目。

git submodule 命令对于大型项目或需要将外部库集成到项目中的情况非常有用。 通过使用子模块，你可以将外部库作为你的项目的一部分来管理，而不必将其直接合并到主仓库中。

## 使用详解
### 1、初始化子模块

`git submodule init`
该命令会初始化配置文件中的所有子模块。它会根据 .gitmodules 文件中的信息设置子模块的 URL 和路径，但不会下载子模块的内容。

常见用法：在克隆了一个包含子模块的仓库后，运行此命令来初始化子模块。
```bash
git clone <repo-url>
cd <repo-dir>
git submodule init
```
### 2、更新子模块

`git submodule update`
该命令会从子模块的远程仓库中拉取子模块的内容，并将其更新到 .gitmodules 文件中指定的提交。

常见用法：在初始化子模块后，或当你需要更新子模块的内容时，运行此命令。

`git submodule update`
### 3、添加子模块 *

`git submodule add <repo-url> [<path>]`
该命令会将指定的 Git 仓库作为子模块添加到当前仓库中。

<repo-url> 是子模块的仓库地址，<path> 是子模块在主仓库中的路径（可选，如果不指定，默认使用子模块仓库的名称作为路径）。

常见用法：将外部库作为子模块添加到项目中。

`git submodule add https://github.com/example/libfoo.git libfoo`
`--branch [tag]`:指定分
### 4、移除子模块

`git submodule deinit [<path>]`
`git rm [<path>]`
`git submodule deinit <path>`：将子模块从 .git/config 文件中移除，并删除子模块目录中的文件。
`git rm <path>`：将子模块的引用从主仓库中删除，并提交更改。
常见用法：从主仓库中移除一个子模块。
```
git submodule deinit libfoo
git rm libfoo
rm -rf .git/modules/libfoo
```
### 5、列出子模块
`git submodule`
列出当前仓库中的所有子模块，以及它们的提交哈希和路径。

常见用法：查看项目中所有子模块的状态。

`git submodule`
### 6、更新所有子模块

`git submodule update --recursive --remote`
`--recursive`：递归地更新所有子模块（包括子模块的子模块）。
`--remote`：从子模块的远程仓库拉取最新的更改。
常见用法：当子模块包含其他子模块时，确保所有层级的子模块都更新到最新版本。

`git submodule update --recursive --remote`
7、检查子模块状态

`git submodule status`
显示子模块的当前状态，包括当前的提交哈希和路径，是否有未提交的更改。

常见用法：查看子模块的当前状态。

`git submodule status`