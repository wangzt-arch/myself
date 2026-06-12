import React, { useEffect, useRef, useState } from "react";
import * as echarts from "echarts";
import china from "./china.json";
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

const scatterData = [
  [10, 8.04, "A"], [8, 6.95, "B"], [13, 7.58, "C"], [9, 8.81, "D"],
  [11, 8.33, "E"], [14, 9.96, "F"], [6, 7.24, "G"], [4, 4.26, "H"],
  [12, 10.84, "I"], [7, 4.82, "J"], [5, 5.68, "K"],
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
    grid: { top: 28, right: 12, bottom: 22, left: 36 },
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
        symbolSize: 6,
        data: monthlyTrend,
        lineStyle: { width: 3, color: "#4de3ff" },
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
    grid: { top: 18, right: 12, bottom: 22, left: 38 },
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
        barWidth: 14,
        data: channelData,
        itemStyle: {
          borderRadius: [6, 6, 0, 0],
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
      textStyle: { color: "rgba(226, 236, 255, 0.78)", fontSize: 11 },
      itemWidth: 8,
      itemHeight: 8,
    },
    series: [
      {
        name: "终端占比",
        type: "pie",
        radius: ["38%", "62%"],
        center: ["50%", "42%"],
        avoidLabelOverlap: true,
        label: {
          color: "#eaf7ff",
          fontSize: 11,
          formatter: "{b}\n{d}%",
        },
        labelLine: {
          lineStyle: { color: "rgba(226, 236, 255, 0.45)" },
        },
        itemStyle: {
          borderColor: "rgba(9, 22, 48, 0.95)",
          borderWidth: 2,
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
      radius: "58%",
      indicator: [
        { name: "性能", max: 100 },
        { name: "体验", max: 100 },
        { name: "内容", max: 100 },
        { name: "稳定", max: 100 },
        { name: "扩展", max: 100 },
      ],
      axisName: { color: "rgba(226, 236, 255, 0.82)", fontSize: 11 },
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
        radius: "82%",
        progress: {
          show: true,
          width: 12,
          itemStyle: { color: "#5bf4ff" },
        },
        axisLine: {
          lineStyle: {
            width: 12,
            color: [[1, "rgba(114, 154, 215, 0.18)"]],
          },
        },
        axisTick: { show: false },
        splitLine: { show: false },
        axisLabel: { show: false },
        pointer: {
          length: "52%",
          width: 4,
          itemStyle: { color: "#ffffff" },
        },
        anchor: {
          show: true,
          size: 10,
          itemStyle: { color: "#ffffff" },
        },
        detail: {
          valueAnimation: true,
          formatter: "{value}%",
          color: "#f4fbff",
          fontSize: 22,
          offsetCenter: [0, "48%"],
        },
        data: [{ value: 74 }],
      },
    ],
  };
}

function buildScatterOption() {
  return {
    grid: { top: 22, right: 14, bottom: 28, left: 36 },
    tooltip: {
      trigger: "item",
      formatter: (params) => {
        return `类别 ${params.data[2]}<br/>X: ${params.data[0]}<br/>Y: ${params.data[1]}`;
      },
    },
    xAxis: {
      type: "value",
      axisLabel,
      splitLine,
      axisLine: { lineStyle: { color: "rgba(142, 181, 230, 0.28)" } },
    },
    yAxis: {
      type: "value",
      axisLabel,
      splitLine,
      axisLine: { lineStyle: { color: "rgba(142, 181, 230, 0.28)" } },
    },
    series: [
      {
        type: "scatter",
        symbolSize: 14,
        data: scatterData,
        itemStyle: {
          color: "#ff7bbd",
          shadowBlur: 12,
          shadowColor: "rgba(255, 123, 189, 0.6)",
        },
        emphasis: {
          itemStyle: {
            color: "#ff4da6",
            borderColor: "#fff",
            borderWidth: 2,
          },
        },
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
  scatter: buildScatterOption,
};

function YqDistribution() {
  const chartRefs = useRef({});
  const [currentTime, setCurrentTime] = useState("");

  useEffect(() => {
    const updateTime = () => {
      const now = new Date();
      setCurrentTime(
        now.toLocaleString("zh-CN", {
          year: "numeric",
          month: "2-digit",
          day: "2-digit",
          hour: "2-digit",
          minute: "2-digit",
          second: "2-digit",
        })
      );
    };
    updateTime();
    const timer = setInterval(updateTime, 1000);
    return () => clearInterval(timer);
  }, []);

  useEffect(() => {
    echarts.registerMap("china", china);

    const charts = Object.keys(optionBuilders)
      .map((key) => {
        const element = chartRefs.current[key];
        if (!element) return null;
        const chart = echarts.init(element);
        chart.setOption(optionBuilders[key]());
        return { key, chart };
      })
      .filter(Boolean);

    const resizeCharts = () => {
      charts.forEach(({ chart }) => chart.resize());
    };

    window.addEventListener("resize", resizeCharts);
    const resizeObserver = new ResizeObserver(resizeCharts);
    Object.values(chartRefs.current).forEach((element) => {
      if (element) resizeObserver.observe(element);
    });

    return () => {
      window.removeEventListener("resize", resizeCharts);
      resizeObserver.disconnect();
      charts.forEach(({ chart }) => chart.dispose());
    };
  }, []);

  return (
    <div className="yq-distribution">
      <main className="chart-dashboard">
        {/* 顶部栏 */}
        <header className="dash-header">
          <h1>数据可视化驾驶舱</h1>
          <span className="dash-time">{currentTime}</span>
        </header>

        {/* 指标行 */}
        <section className="chart-metrics" aria-label="关键指标">
          {metrics.map((item) => (
            <article className="metric-card" key={item.label}>
              <span>{item.label}</span>
              <strong>{item.value}</strong>
              <em>{item.delta}</em>
            </article>
          ))}
        </section>

        {/* 主体内容区：左3 | 中地图 | 右3 */}
        <section className="dash-body">
          {/* 左侧 3 个图表 */}
          <div className="dash-left">
            <article className="dash-grid-item">
              <div className="chart-card__head">
                <div>
                  <h2>访问趋势</h2>
                  <p>近 7 月增长曲线</p>
                </div>
                <i />
              </div>
              <div
                className="chart-canvas"
                ref={(el) => { chartRefs.current.line = el; }}
              />
            </article>
            <article className="dash-grid-item">
              <div className="chart-card__head">
                <div>
                  <h2>渠道贡献</h2>
                  <p>核心来源排名</p>
                </div>
                <i />
              </div>
              <div
                className="chart-canvas"
                ref={(el) => { chartRefs.current.bar = el; }}
              />
            </article>
            <article className="dash-grid-item">
              <div className="chart-card__head">
                <div>
                  <h2>终端占比</h2>
                  <p>访问设备结构</p>
                </div>
                <i />
              </div>
              <div
                className="chart-canvas"
                ref={(el) => { chartRefs.current.pie = el; }}
              />
            </article>
          </div>

          {/* 中间地图 */}
          <article className="dash-map">
            <div className="chart-card__head">
              <div>
                <h2>全国热力分布</h2>
                <p>实时城市活跃度</p>
              </div>
              <i />
            </div>
            <div
              className="chart-canvas"
              ref={(el) => { chartRefs.current.map = el; }}
            />
          </article>

          {/* 右侧 3 个图表 */}
          <div className="dash-left">
            <article className="dash-grid-item">
              <div className="chart-card__head">
                <div>
                  <h2>能力雷达</h2>
                  <p>综合表现评估</p>
                </div>
                <i />
              </div>
              <div
                className="chart-canvas"
                ref={(el) => { chartRefs.current.radar = el; }}
              />
            </article>
            <article className="dash-grid-item">
              <div className="chart-card__head">
                <div>
                  <h2>转化效率</h2>
                  <p>当前目标完成度</p>
                </div>
                <i />
              </div>
              <div
                className="chart-canvas"
                ref={(el) => { chartRefs.current.gauge = el; }}
              />
            </article>
            <article className="dash-grid-item">
              <div className="chart-card__head">
                <div>
                  <h2>分布散点</h2>
                  <p>样本相关性分析</p>
                </div>
                <i />
              </div>
              <div
                className="chart-canvas"
                ref={(el) => { chartRefs.current.scatter = el; }}
              />
            </article>
          </div>
        </section>
      </main>
    </div>
  );
}

export default YqDistribution;
