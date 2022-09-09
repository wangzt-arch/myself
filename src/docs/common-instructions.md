# 常用指令

## 1.包管理
~~~js
npm config get registry

npm config set registry https://registry.npm.taobao.org

yarn config set registry https://registry.npm.taobao.org
~~~
## 2.vite
### 本地服务
~~~js
vite --host xxx.xxx.x.xxx
~~~
## 3.git
### 克隆
~~~js
git clone url
~~~
### 初始化
~~~js
git init 
~~~
### 暂存
~~~js
git add .
~~~
### 提交
~~~js
git commit -m "xxx"
~~~
### 推送
~~~js
git push remoteName branchName
~~~
### 拉取
~~~js
git pull remoteName branchName
~~~
### 添加仓库
~~~js
git remote add remoteName url
~~~
### 移除仓库
~~~js
git remote remove remoteName url
~~~
### 创建并切换分支
~~~js
git checkout -b branchName
~~~
### 查看所有分支
~~~js
git branch -a
~~~
### 查看所有仓库
~~~js
git remote -v
~~~