// ============================================================
// commitlint 中文提交规范
// 格式:
//   type(scope): 中文描述
//
//   type 取值：
//     feat     新功能
//     fix      修复 bug
//     style    代码格式调整（不影响逻辑，空格/分号/缩进等）
//     refactor 代码重构（不是新功能也不是修 bug）
//     perf     性能优化
//     docs     文档变更
//     test     增加或调整测试
//     build    构建工具/依赖升级/脚手架变更
//     ci       CI/CD 配置变更
//     chore    杂项（.gitignore、配置、示例数据等）
//     revert   回滚之前的提交
//
//   示例：
//     feat(cesium): 新增城市点击弹窗
//     fix(home): 修复首页导航栏在移动端溢出
//     docs: 补充 AI 开发规范文档
//     chore: 配置 husky 与 commitlint
// ============================================================

module.exports = {
  extends: [],
  rules: {
    // —— type 必须在下面枚举中
    "type-enum": [
      2,
      "always",
      [
        "feat",
        "fix",
        "style",
        "refactor",
        "perf",
        "docs",
        "test",
        "build",
        "ci",
        "chore",
        "revert"
      ]
    ],

    // —— type 必须小写
    "type-case": [2, "always", "lower-case"],

    // —— type 不能为空
    "type-empty": [2, "never"],

    // —— subject（冒号后面的描述）不能为空
    "subject-empty": [2, "never"],

    // —— subject 必须是中文（至少包含一个中文字符）
    "subject-chinese": [2, "always"],

    // —— subject 不以句号结尾
    "subject-full-stop": [2, "never", "."],

    // —— 整个 header 不超过 100 字符
    "header-max-length": [2, "always", 100],

    // —— colon 后面必须有空格（如 feat: xxx）
    "subject-space-before": [2, "always"]
  },

  plugins: [
    {
      rules: {
        "subject-chinese": ({ subject }) => {
          if (!subject) {
            return [false, "提交描述（subject）不能为空，且必须用中文"];
          }
          const hasChinese = /[\u4e00-\u9fa5]/.test(subject);
          if (!hasChinese) {
            return [false, "提交描述必须使用中文（至少包含一个中文字符）"];
          }
          return [true];
        },
        "subject-space-before": ({ raw }) => {
          // 确保 type 之后有 ": "（冒号 + 空格）
          const ok = /^[a-z]+(\([^)]+\))?:\s+/.test(raw);
          if (!ok) {
            return [false, "type 后必须跟「: 」（冒号 + 空格），例如：feat(cesium): 新增城市标记"];
          }
          return [true];
        }
      }
    }
  ]
};
