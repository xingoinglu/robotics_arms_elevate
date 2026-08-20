# GitHub 代码上传流程

本文介绍如何将本地代码上传到 GitHub，包括环境配置、首次上传、日常更新、分支协作和常见问题处理。

> 当前仓库已经配置远程仓库 `origin`，地址为
> `git@github.com:xingoinglu/robotics_arms_elevate.git`，主分支为 `main`。

## 1. 上传前准备

### 1.1 安装并检查 Git

```bash
git --version
```

如果尚未设置提交者信息，请执行：

```bash
git config --global user.name "你的 GitHub 用户名"
git config --global user.email "你的 GitHub 邮箱"
```

检查配置：

```bash
git config --global --list
```

### 1.2 配置 GitHub 身份验证

推荐使用 SSH。先检查本机是否已有公钥：

```bash
ls ~/.ssh/id_ed25519.pub
```

如果没有，则创建 SSH 密钥：

```bash
ssh-keygen -t ed25519 -C "你的 GitHub 邮箱"
```

查看并复制公钥：

```bash
cat ~/.ssh/id_ed25519.pub
```

然后进入 GitHub：

1. 点击头像，进入 **Settings**。
2. 进入 **SSH and GPG keys**。
3. 点击 **New SSH key**。
4. 粘贴公钥并保存。

测试连接：

```bash
ssh -T git@github.com
```

首次连接时输入 `yes`。如果出现 GitHub 的认证成功提示，说明 SSH 配置完成。

## 2. 当前仓库的日常上传流程

进入仓库目录：

```bash
cd /home/xie/elevate_ws/src
```

### 第一步：拉取远程最新代码

```bash
git pull --rebase origin main
```

`--rebase` 可以让本地提交历史保持整洁。如果当前有尚未提交的修改，Git 可能要求先提交或临时保存修改。

### 第二步：查看修改内容

```bash
git status
git diff
```

确认以下内容没有被误加入：

- 密码、Token、私钥和 `.env` 文件；
- ROS 2 的 `build/`、`install/` 和 `log/` 目录；
- 模型、数据集、录像等大文件；
- 与本次任务无关的临时文件。

### 第三步：添加需要提交的文件

推荐明确指定文件：

```bash
git add 路径/文件名
```

例如：

```bash
git add GITHUB_UPLOAD_GUIDE.md
```

如果确认当前目录下的所有修改都需要提交，也可以执行：

```bash
git add .
```

再次检查即将提交的内容：

```bash
git status
git diff --cached
```

### 第四步：创建本地提交

```bash
git commit -m "docs: 添加 GitHub 代码上传流程"
```

建议提交信息使用“类型 + 修改内容”的格式，例如：

```text
feat: 添加电梯按钮识别功能
fix: 修复机械臂轨迹执行异常
docs: 更新项目使用说明
refactor: 重构视觉处理模块
test: 添加控制器测试
chore: 更新构建配置
```

### 第五步：推送到 GitHub

```bash
git push origin main
```

推送完成后，打开 GitHub 仓库页面，确认最新提交和文件已经出现。

### 日常上传命令汇总

```bash
cd /home/xie/elevate_ws/src
git pull --rebase origin main
git status
git diff
git add 路径/文件名
git diff --cached
git commit -m "类型: 简要说明修改内容"
git push origin main
```

## 3. 新项目首次上传到 GitHub

### 3.1 在 GitHub 创建空仓库

登录 GitHub，点击 **New repository**，填写仓库名称并创建仓库。

如果本地已经有 `README.md`、`.gitignore` 或 `LICENSE`，创建远程仓库时不要重复初始化这些文件，以免首次推送产生冲突。

### 3.2 初始化本地仓库

```bash
cd /你的/项目目录
git init
git branch -M main
```

### 3.3 检查 `.gitignore`

提交前应创建 `.gitignore`，排除构建产物、缓存、密钥和本机配置。例如，ROS 2/Python 项目通常需要排除：

```gitignore
build/
install/
log/
__pycache__/
*.pyc
.env
.venv/
.vscode/
.idea/
```

### 3.4 创建首次提交

```bash
git add .
git status
git commit -m "chore: 初始化项目"
```

### 3.5 关联远程仓库并推送

使用 SSH 地址：

```bash
git remote add origin git@github.com:你的用户名/你的仓库名.git
git push -u origin main
```

设置 `-u` 后，后续在当前分支可直接执行：

```bash
git pull --rebase
git push
```

检查远程仓库地址：

```bash
git remote -v
```

如果 `origin` 地址填写错误，可以修改：

```bash
git remote set-url origin git@github.com:你的用户名/你的仓库名.git
```

## 4. 推荐的分支协作流程

多人协作或开发新功能时，不建议直接修改 `main`，可以创建功能分支：

```bash
git switch main
git pull --rebase origin main
git switch -c feature/功能名称
```

完成修改后提交并推送：

```bash
git add 路径/文件名
git commit -m "feat: 添加某项功能"
git push -u origin feature/功能名称
```

然后在 GitHub 页面创建 Pull Request，代码检查通过后再合并到 `main`。

## 5. 常见问题

### 5.1 推送被拒绝：远程仓库有新提交

先同步远程代码，再重新推送：

```bash
git pull --rebase origin main
git push origin main
```

如果发生冲突，打开冲突文件，处理 `<<<<<<<`、`=======` 和 `>>>>>>>` 标记，然后执行：

```bash
git add 冲突文件
git rebase --continue
git push origin main
```

如果决定放弃本次 rebase：

```bash
git rebase --abort
```

### 5.2 当前有未提交修改，无法拉取

可以先提交修改，或者临时保存：

```bash
git stash push -m "临时保存"
git pull --rebase origin main
git stash pop
```

执行 `stash pop` 后仍需检查是否产生冲突。

### 5.3 SSH 提示 `Permission denied (publickey)`

依次检查：

```bash
ssh -T git@github.com
ssh-add -l
git remote -v
```

确认公钥已经添加到正确的 GitHub 账号，并确认远程地址使用的是 `git@github.com:...` 格式。

### 5.4 文件超过 GitHub 大小限制

不要直接提交模型、数据集、录像或其他大文件。此类文件可以使用 Git LFS 或独立的文件存储服务管理。

如果文件已经被 `git add`，但还没有提交，可以取消暂存：

```bash
git restore --staged 路径/大文件
```

然后将对应路径加入 `.gitignore`。

### 5.5 子模块没有完整上传

本仓库包含 Git 子模块配置。克隆含子模块的仓库时，建议使用：

```bash
git clone --recurse-submodules 仓库地址
```

已经完成普通克隆时，可以执行：

```bash
git submodule update --init --recursive
```

父仓库只记录子模块的提交位置。修改子模块后，需要先在子模块自己的仓库中提交并推送，再回到父仓库提交子模块引用的变化。

## 6. 上传前检查清单

- [ ] 已执行 `git pull --rebase` 同步远程代码。
- [ ] 已使用 `git status` 和 `git diff` 检查修改。
- [ ] 没有提交密码、Token、私钥或个人配置。
- [ ] 没有提交构建目录、缓存和不必要的大文件。
- [ ] `git diff --cached` 中只有本次需要上传的内容。
- [ ] 提交信息能够清楚说明本次修改。
- [ ] 推送后已在 GitHub 页面确认结果。

