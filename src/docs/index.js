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
import aiHotTerms from "./ai-hot-terms.md";
import jsInterview from "./js-interview.md";
import htmlInterview from "./html-interview.md";

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
    title: "AI 圈子热词百科",
    category: "AI 工具",
    tags: ["AI", "术语", "百科"],
    value: aiHotTerms,
  },
  {
    title: "常用正则",
    category: "前端基础",
    tags: ["Regex", "JavaScript"],
    value: regex,
  },
  {
    title: "ES6+ 新特性概览",
    category: "前端基础",
    tags: ["ES6", "JavaScript", "ES2020", "ES2022"],
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
  {
    title: "JS 面试题",
    category: "面试相关",
    tags: ["JavaScript", "面试", "基础", "进阶"],
    value: jsInterview,
  },
  {
    title: "HTML 面试题",
    category: "面试相关",
    tags: ["HTML", "面试", "基础", "进阶"],
    value: htmlInterview,
  },
];

export default docs;
