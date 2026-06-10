// ============================================================
// commitizen 中文交互式提交适配器
// 使用: npm run commit
//   或  : npx git-cz
// ============================================================

const typeList = [
  {
    value: "feat",
    name: "feat:     新功能 / 新增页面 / 新增组件 / 新增能力",
    hint: "  例如： feat(cesium): 新增城市点击弹窗"
  },
  {
    value: "fix",
    name: "fix:      修复 bug / 修复报错 / 修复逻辑错误",
    hint: "  例如： fix(home): 修复首页导航栏在移动端溢出"
  },
  {
    value: "style",
    name: "style:    代码格式调整（不影响逻辑：空格/缩进/分号等）",
    hint: "  例如： style: 统一组件 import 顺序"
  },
  {
    value: "refactor",
    name: "refactor: 代码重构（非新功能也非修 bug）",
    hint: "  例如： refactor(model-preview): 抽取公共模型加载逻辑"
  },
  {
    value: "perf",
    name: "perf:     性能优化",
    hint: "  例如： perf(cesium): 优化城市标记实体渲染数量"
  },
  {
    value: "docs",
    name: "docs:     文档变更（README / 注释 / Markdown）",
    hint: "  例如： docs: 补充 README 使用说明"
  },
  {
    value: "test",
    name: "test:     新增或调整测试用例",
    hint: "  例如： test: 补充 ECharts 图表渲染测试"
  },
  {
    value: "build",
    name: "build:    构建工具/依赖升级/脚手架变更",
    hint: "  例如： build(deps): 升级 cesium 至 1.141"
  },
  {
    value: "ci",
    name: "ci:       CI/CD 配置变更",
    hint: "  例如： ci(workflow): 新增部署工作流"
  },
  {
    value: "chore",
    name: "chore:    杂项（.gitignore/配置/示例数据等）",
    hint: "  例如： chore: 配置 husky 与 commitlint"
  },
  {
    value: "revert",
    name: "revert:   回滚之前的提交",
    hint: "  例如： revert: 回滚 xxx 功能"
  }
];

const scopeList = [
  { value: "", name: "(无) - 不指定模块" },
  { value: "home", name: "home  - 首页" },
  { value: "docs", name: "docs  - 文档中心" },
  { value: "about", name: "about - 关于页" },
  { value: "cesium", name: "cesium - 3D 地球" },
  { value: "model-preview", name: "model-preview - 3D 模型预览" },
  { value: "logicflow", name: "logicflow - 流程图" },
  { value: "chart", name: "chart  - 图表 (yq-distribution)" },
  { value: "translate", name: "translate - 翻译" },
  { value: "video", name: "video  - 视频案例" },
  { value: "header", name: "header - 导航栏" },
  { value: "api", name: "api    - 接口封装" },
  { value: "utils", name: "utils  - 工具函数" },
  { value: "hooks", name: "hooks  - React Hooks" },
  { value: "config", name: "config - 构建/Webpack 配置" },
  { value: "deps", name: "deps   - 依赖" }
];

function pad(str, len, char = " ") {
  return str + char.repeat(Math.max(0, len - str.length));
}

module.exports = {
  prompter(cz, commit) {
    const maxTypeLen = Math.max(...typeList.map(t => t.value.length));

    cz.prompt([
      {
        type: "list",
        name: "type",
        message: "请选择本次提交的类型（必选）",
        choices: typeList.map(t => ({
          name: `${pad(t.value, maxTypeLen)}  ${t.hint.trim()}`,
          value: t.value
        }))
      },
      {
        type: "list",
        name: "scope",
        message: "请选择影响的模块 scope（可选，可手动输入）",
        choices: scopeList.map(s => ({ name: s.name, value: s.value }))
      },
      {
        type: "input",
        name: "subject",
        message: "请用简短中文描述本次改动（不超过 80 字，必填）\n  > ",
        filter(value) {
          return (value || "").trim();
        },
        validate(value) {
          if (!value) {
            return "描述不能为空，请用中文说明本次改动";
          }
          if (value.length > 80) {
            return `描述过长（${value.length} 字），请控制在 80 字以内，详情可以写在 body`;
          }
          if (!/[\u4e00-\u9fa5]/.test(value)) {
            return "描述必须包含中文（至少一个中文字符）";
          }
          return true;
        }
      },
      {
        type: "input",
        name: "body",
        message: "详细描述本次改动（可选，可直接回车跳过）\n  > ",
        filter(value) {
          return (value || "").trim();
        }
      },
      {
        type: "confirm",
        name: "confirm",
        message: (answers) => {
          const scope = answers.scope ? `(${answers.scope})` : "";
          const header = `${answers.type}${scope}: ${answers.subject}`;
          const body = answers.body ? `\n\n${answers.body}` : "";
          return `\n\n———— 提交信息预览 ————\n\n${header}${body}\n\n\n确认提交？`;
        },
        default: true
      }
    ]).then((answers) => {
      if (!answers.confirm) {
        console.log("\n❌ 已取消提交。");
        process.exit(1);
      }

      const scope = answers.scope ? `(${answers.scope})` : "";
      const header = `${answers.type}${scope}: ${answers.subject}`;
      const message = answers.body ? `${header}\n\n${answers.body}` : header;

      commit(message);
    });
  }
};
