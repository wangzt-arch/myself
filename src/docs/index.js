import regex from "./regex.md";
import fp from "./fp.md";
import harmony from "./harmonyos.md";
import lets from "./let.md";
import utils from "./utils.md";
import skills from "./skills.md";
import commonInstructions from "./common-instructions.md";
import openlayers from "./openlayers-func.md";
import cesium from "./cesium.md";
import ai from "./ai.md";
import codexStart from "./codex-start.md";
import aiPrompts from "./ai-prompts.md";

const docs = [
  {
    title: "AI 与 Codex 入门",
    category: "AI 工具",
    tags: ["AI", "Codex", "Prompt"],
    value: ai,
  },
  {
    title: "Codex 下载注册指南",
    category: "AI 工具",
    tags: ["Codex", "安装", "登录"],
    value: codexStart,
  },
  {
    title: "AI 提示语最佳实践",
    category: "AI 工具",
    tags: ["AI", "Prompt", "最佳实践"],
    value: aiPrompts,
  },
  {
    title: "常用正则",
    category: "前端基础",
    tags: ["Regex", "JavaScript"],
    value: regex,
  },
  {
    title: "变量声明",
    category: "前端基础",
    tags: ["ES6", "JavaScript"],
    value: lets,
  },
  {
    title: "函数式编程",
    category: "前端基础",
    tags: ["JavaScript", "FP"],
    value: fp,
  },
  {
    title: "常用 JS 工具",
    category: "工程效率",
    tags: ["Utils", "Browser"],
    value: utils,
  },
  {
    title: "鸿蒙 OS",
    category: "移动端",
    tags: ["HarmonyOS"],
    value: harmony,
  },
  {
    title: "常用指令",
    category: "工程效率",
    tags: ["Git", "NPM", "Vite"],
    value: commonInstructions,
  },
  {
    title: "掌握技能",
    category: "能力地图",
    tags: ["Frontend", "Skill"],
    value: skills,
  },
  {
    title: "OpenLayers 方法",
    category: "GIS 可视化",
    tags: ["OpenLayers", "GIS"],
    value: openlayers,
  },
  {
    title: "Cesium 方法",
    category: "GIS 可视化",
    tags: ["Cesium", "3D GIS"],
    value: cesium,
  },
];

export default docs;
