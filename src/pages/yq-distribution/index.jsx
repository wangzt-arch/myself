import React, { useEffect, useRef } from "react";
import * as echarts from "echarts";
import china from "./china.json";
import Header from "../../components/Header";
import "./index.scss";

const cityScatterData = [
  { name: "北京", value: [116.41, 39.92, 120] },
  { name: "上海", value: [121.47, 31.23, 92] },
  { name: "杭州", value: [120.16, 30.28, 76] },
  { name: "深圳", value: [114.05, 22.55, 88] },
  { name: "成都", value: [104.06, 30.67, 64] },
  { name: "武汉", value: [114.31, 30.52, 58] },
  { name: "西安", value: [108.94, 34.34, 46] },
];

const monthlyTrend = [42, 58, 64, 72, 95, 116, 132];
const channelData = [86, 72, 68, 52, 44];
const deviceData = [
  { name: "Web", value: 42 },
  { name: "Mobile", value: 34 },
  { name: "Desktop", value: 16 },
  { name: "API", value: 8 },
];

const chartConfigs = [
  {
    key: "map",
    title: "全国热力分布",
    subtitle: "实时城市活跃度",
    className: "chart-card chart-card--wide chart-card--map",
  },
  {
    key: "line",
    title: "访问趋势",
    subtitle: "近 7 月增长曲线",
    className: "chart-card",
  },
  {
    key: "bar",
    title: "渠道贡献",
    subtitle: "核心来源排名",
    className: "chart-card",
  },
  {
    key: "pie",
    title: "终端占比",
    subtitle: "访问设备结构",
    className: "chart-card",
  },
  {
    key: "radar",
    title: "能力雷达",
    subtitle: "综合表现评估",
    className: "chart-card",
  },
  {
    key: "gauge",
    title: "转化效率",
    subtitle: "当前目标完成度",
    className: "chart-card",
  },
];

const metrics = [
  { label: "总访问量", value: "128.6k", delta: "+18.4%" },
  { label: "活跃城市", value: "86", delta: "+12" },
  { label: "转化率", value: "73.8%", delta: "+6.2%" },
  { label: "平均响应", value: "128ms", delta: "-24ms" },
];

const axisLabel = {
  color: "rgba(226, 236, 255, 0.72)",
  fontSize: 11,
};

const splitLine = {
  lineStyle: {
    color: "rgba(116, 152, 207, 0.16)",
  },
};

function buildMapOption() {
  return {
    backgroundColor: "transparent",
    tooltip: {
      trigger: "item",
      borderWidth: 0,
      backgroundColor: "rgba(7, 18, 38, 0.92)",
      textStyle: { color: "#e8f4ff" },
      formatter: (params) => {
        const value = params.value?.[2] ?? 0;
        return `${params.name}<br/>活跃度：${value}`;
      },
    },
    geo: {
      map: "china",
      roam: true,
      zoom: 1.18,
      label: { show: false },
      itemStyle: {
        areaColor: "rgba(18, 54, 103, 0.82)",
        borderColor: "rgba(96, 198, 255, 0.8)",
        borderWidth: 0.7,
        shadowColor: "rgba(18, 202, 255, 0.55)",
        shadowBlur: 18,
      },
      emphasis: {
        label: { show: false },
        itemStyle: {
          areaColor: "rgba(47, 145, 255, 0.9)",
        },
      },
    },
    series: [
      {
        name: "城市活跃度",
        type: "effectScatter",
        coordinateSystem: "geo",
        data: cityScatterData,
        symbolSize: (value) => Math.max(value[2] / 7, 8),
        showEffectOn: "render",
        rippleEffect: {
          brushType: "stroke",
          scale: 4.2,
        },
        label: {
          show: true,
          position: "right",
          color: "#d9fbff",
          fontSize: 12,
          formatter: "{b}",
        },
        itemStyle: {
          color: "#54f0ff",
          shadowBlur: 22,
          shadowColor: "#54f0ff",
        },
        zlevel: 2,
      },
    ],
  };
}

function buildLineOption() {
  return {
    grid: { top: 30, right: 16, bottom: 28, left: 38 },
    tooltip: { trigger: "axis" },
    xAxis: {
      type: "category",
      boundaryGap: false,
      data: ["1月", "2月", "3月", "4月", "5月", "6月", "7月"],
      axisLabel,
      axisLine: { lineStyle: { color: "rgba(142, 181, 230, 0.28)" } },
    },
    yAxis: {
      type: "value",
      axisLabel,
      splitLine,
    },
    series: [
      {
        name: "访问量",
        type: "line",
        smooth: true,
        symbolSize: 8,
        data: monthlyTrend,
        lineStyle: { width: 4, color: "#4de3ff" },
        itemStyle: { color: "#ffffff", borderColor: "#4de3ff", borderWidth: 2 },
        areaStyle: {
          color: new echarts.graphic.LinearGradient(0, 0, 0, 1, [
            { offset: 0, color: "rgba(77, 227, 255, 0.45)" },
            { offset: 1, color: "rgba(77, 227, 255, 0.02)" },
          ]),
        },
      },
    ],
  };
}

function buildBarOption() {
  return {
    grid: { top: 22, right: 18, bottom: 28, left: 42 },
    tooltip: { trigger: "axis" },
    xAxis: {
      type: "category",
      data: ["搜索", "直达", "文档", "社媒", "外链"],
      axisLabel,
      axisLine: { lineStyle: { color: "rgba(142, 181, 230, 0.28)" } },
    },
    yAxis: {
      type: "value",
      axisLabel,
      splitLine,
    },
    series: [
      {
        name: "贡献值",
        type: "bar",
        barWidth: 18,
        data: channelData,
        itemStyle: {
          borderRadius: [8, 8, 0, 0],
          color: new echarts.graphic.LinearGradient(0, 0, 0, 1, [
            { offset: 0, color: "#8fffca" },
            { offset: 0.5, color: "#3fdcff" },
            { offset: 1, color: "#4269ff" },
          ]),
        },
      },
    ],
  };
}

function buildPieOption() {
  return {
    tooltip: { trigger: "item" },
    legend: {
      bottom: 0,
      textStyle: { color: "rgba(226, 236, 255, 0.78)" },
      itemWidth: 10,
      itemHeight: 10,
    },
    series: [
      {
        name: "终端占比",
        type: "pie",
        radius: ["42%", "68%"],
        center: ["50%", "44%"],
        avoidLabelOverlap: true,
        label: {
          color: "#eaf7ff",
          formatter: "{b}\n{d}%",
        },
        labelLine: {
          lineStyle: { color: "rgba(226, 236, 255, 0.45)" },
        },
        itemStyle: {
          borderColor: "rgba(9, 22, 48, 0.95)",
          borderWidth: 3,
        },
        data: deviceData,
      },
    ],
    color: ["#42e8ff", "#9cff6d", "#ffcf5c", "#ff7bbd"],
  };
}

function buildRadarOption() {
  return {
    radar: {
      radius: "64%",
      indicator: [
        { name: "性能", max: 100 },
        { name: "体验", max: 100 },
        { name: "内容", max: 100 },
        { name: "稳定", max: 100 },
        { name: "扩展", max: 100 },
      ],
      axisName: { color: "rgba(226, 236, 255, 0.82)" },
      splitLine,
      splitArea: {
        areaStyle: {
          color: ["rgba(58, 124, 255, 0.05)", "rgba(58, 124, 255, 0.12)"],
        },
      },
      axisLine: { lineStyle: { color: "rgba(116, 152, 207, 0.2)" } },
    },
    series: [
      {
        type: "radar",
        data: [
          {
            value: [88, 82, 76, 91, 84],
            name: "当前表现",
            areaStyle: { color: "rgba(99, 245, 255, 0.25)" },
            lineStyle: { color: "#63f5ff", width: 2 },
            itemStyle: { color: "#63f5ff" },
          },
        ],
      },
    ],
  };
}

function buildGaugeOption() {
  return {
    series: [
      {
        type: "gauge",
        startAngle: 210,
        endAngle: -30,
        min: 0,
        max: 100,
        radius: "88%",
        progress: {
          show: true,
          width: 16,
          itemStyle: { color: "#5bf4ff" },
        },
        axisLine: {
          lineStyle: {
            width: 16,
            color: [[1, "rgba(114, 154, 215, 0.18)"]],
          },
        },
        axisTick: { show: false },
        splitLine: { show: false },
        axisLabel: { show: false },
        pointer: {
          length: "58%",
          width: 5,
          itemStyle: { color: "#ffffff" },
        },
        anchor: {
          show: true,
          size: 12,
          itemStyle: { color: "#ffffff" },
        },
        detail: {
          valueAnimation: true,
          formatter: "{value}%",
          color: "#f4fbff",
          fontSize: 28,
          offsetCenter: [0, "48%"],
        },
        data: [{ value: 74 }],
      },
    ],
  };
}

const optionBuilders = {
  map: buildMapOption,
  line: buildLineOption,
  bar: buildBarOption,
  pie: buildPieOption,
  radar: buildRadarOption,
  gauge: buildGaugeOption,
};

function YqDistribution() {
  const chartRefs = useRef({});

  useEffect(() => {
    echarts.registerMap("china", china);

    const charts = chartConfigs
      .map(({ key }) => {
        const element = chartRefs.current[key];
        if (!element) {
          return null;
        }

        const chart = echarts.init(element);
        chart.setOption(optionBuilders[key]());
        return chart;
      })
      .filter(Boolean);

    const resizeCharts = () => {
      charts.forEach((chart) => chart.resize());
    };

    window.addEventListener("resize", resizeCharts);
    const resizeObserver = new ResizeObserver(resizeCharts);
    Object.values(chartRefs.current).forEach((element) => {
      if (element) {
        resizeObserver.observe(element);
      }
    });

    return () => {
      window.removeEventListener("resize", resizeCharts);
      resizeObserver.disconnect();
      charts.forEach((chart) => chart.dispose());
    };
  }, []);

  return (
    <div className="yq-distribution">
      <Header />
      <main className="chart-dashboard">
        <section className="chart-hero">
          <div>
            <p className="chart-eyebrow">ECharts Dashboard</p>
            <h1>数据可视化驾驶舱</h1>
            <p className="chart-hero__desc">
              以地图为核心，组合趋势、渠道、占比与能力评估，让页面从单一图表升级为可浏览的动态分析面板。
            </p>
          </div>
          <div className="chart-pulse" aria-hidden="true">
            <span />
            <span />
            <span />
          </div>
        </section>

        <section className="chart-metrics" aria-label="关键指标">
          {metrics.map((item) => (
            <article className="metric-card" key={item.label}>
              <span>{item.label}</span>
              <strong>{item.value}</strong>
              <em>{item.delta}</em>
            </article>
          ))}
        </section>

        <section className="chart-grid">
          {chartConfigs.map((item) => (
            <article className={item.className} key={item.key}>
              <div className="chart-card__head">
                <div>
                  <h2>{item.title}</h2>
                  <p>{item.subtitle}</p>
                </div>
                <i />
              </div>
              <div
                className="chart-canvas"
                ref={(element) => {
                  chartRefs.current[item.key] = element;
                }}
              />
            </article>
          ))}
        </section>
      </main>
    </div>
  );
}

export default YqDistribution;
