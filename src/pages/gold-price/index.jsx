import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import * as echarts from 'echarts/core';
import { CanvasRenderer } from 'echarts/renderers';
import { LineChart, ScatterChart } from 'echarts/charts';
import { TooltipComponent, GridComponent } from 'echarts/components';
import {
  fetchDomesticBars,
  fetchDomesticLatest,
  fetchIntlBars,
  fetchIntlQuote,
  isHistoryFresh,
  localDateKey,
  readCachedBars,
  readCachedIntlBars,
  readFilter,
  saveCachedBars,
  saveCachedIntlBars,
  saveFilter,
} from './api';
import {
  DEFAULT_FILTER,
  DIRECTION_OPTIONS,
  GRANULARITY_OPTIONS,
  RANGE_OPTIONS,
  aggregateBars,
  filterByChange,
  filterByRange,
  formatDate,
  formatPercent,
  formatPrice,
  toCsv,
  withChange,
} from './utils';
import './index.scss';

echarts.use([CanvasRenderer, LineChart, ScatterChart, TooltipComponent, GridComponent]);

const PAGE_SIZE = 10;

function barLabel(bar) {
  return bar.label || formatDate(bar.date);
}

function formatUpdatedAt(iso) {
  if (!iso) return '—';
  const date = new Date(iso);
  if (Number.isNaN(date.getTime())) return '—';
  return date.toLocaleString('zh-CN', {
    month: '2-digit',
    day: '2-digit',
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
  });
}

export default function GoldPrice() {
  const initialCache = useMemo(readCachedBars, []);
  const initialAt = useRef(initialCache.at);

  const [history, setHistory] = useState(initialCache);
  const [status, setStatus] = useState(initialCache.bars.length ? 'ready' : 'loading');
  const [intlQuote, setIntlQuote] = useState(null);
  const [intlError, setIntlError] = useState('');
  const initialIntlCache = useMemo(readCachedIntlBars, []);
  const initialIntlAt = useRef(initialIntlCache.at);
  const [intlHistory, setIntlHistory] = useState(initialIntlCache);
  const [intlBarsStatus, setIntlBarsStatus] = useState(
    initialIntlCache.bars.length ? 'ready' : 'loading'
  );
  const [intlBarsError, setIntlBarsError] = useState('');
  const [domesticLatest, setDomesticLatest] = useState(null);
  const [domesticError, setDomesticError] = useState('');
  const [filter, setFilter] = useState(() => ({ ...DEFAULT_FILTER, ...(readFilter() || {}) }));
  const [page, setPage] = useState(1);

  const chartElement = useRef(null);
  const chartInstance = useRef(null);
  const intlChartElement = useRef(null);
  const intlChartInstance = useRef(null);

  const loadIntl = useCallback(async () => {
    try {
      const quote = await fetchIntlQuote();
      setIntlQuote(quote);
      setIntlError('');
    } catch (err) {
      setIntlError(err.message || '国际报价获取失败');
    }
  }, []);

  const loadIntlBars = useCallback(async () => {
    try {
      const bars = await fetchIntlBars();
      saveCachedIntlBars(bars);
      setIntlHistory({ bars, at: Date.now() });
      setIntlBarsError('');
      setIntlBarsStatus('ready');
    } catch (err) {
      setIntlBarsError(err.message || '国际历史行情加载失败');
      setIntlBarsStatus((prev) => (prev === 'ready' ? 'ready' : 'error'));
    }
  }, []);

  const loadDomesticBars = useCallback(async () => {
    try {
      const bars = await fetchDomesticBars();
      saveCachedBars(bars);
      setHistory({ bars, at: Date.now() });
      setDomesticError('');
      setStatus('ready');
    } catch (err) {
      setDomesticError(err.message || '国内历史行情加载失败');
      setStatus((prev) => (prev === 'ready' ? 'ready' : 'error'));
    }
  }, []);

  const loadDomesticLatest = useCallback(async () => {
    try {
      const latest = await fetchDomesticLatest();
      if (latest) {
        setDomesticLatest(latest);
        setDomesticError('');
      }
    } catch (err) {
      setDomesticError(err.message || '国内最新价获取失败');
    }
  }, []);

  useEffect(() => {
    loadIntl();
    const timer = setInterval(() => {
      if (!document.hidden) loadIntl();
    }, 30000);
    const handleVisibility = () => {
      if (!document.hidden) loadIntl();
    };
    document.addEventListener('visibilitychange', handleVisibility);
    return () => {
      clearInterval(timer);
      document.removeEventListener('visibilitychange', handleVisibility);
    };
  }, [loadIntl]);

  useEffect(() => {
    if (!isHistoryFresh(initialAt.current)) loadDomesticBars();
    loadDomesticLatest();
    const timer = setInterval(() => {
      if (!document.hidden) loadDomesticLatest();
    }, 60000);
    const handleVisibility = () => {
      if (!document.hidden) loadDomesticLatest();
    };
    document.addEventListener('visibilitychange', handleVisibility);
    return () => {
      clearInterval(timer);
      document.removeEventListener('visibilitychange', handleVisibility);
    };
  }, [loadDomesticBars, loadDomesticLatest]);

  useEffect(() => {
    if (!isHistoryFresh(initialIntlAt.current)) loadIntlBars();
  }, [loadIntlBars]);

  useEffect(() => {
    saveFilter(filter);
    setPage(1);
  }, [filter]);

  const bars = history.bars;
  const intlBars = intlHistory.bars;
  // 日线接口收盘前不含当天，把实时行情合并成今日 K 线
  const liveBars = useMemo(() => {
    if (!domesticLatest || !domesticLatest.date || !bars.length) return bars;
    const last = bars[bars.length - 1];
    if (domesticLatest.date < last.date) return bars;
    if (domesticLatest.date === last.date) {
      return [
        ...bars.slice(0, -1),
        {
          ...last,
          high: Math.max(last.high, domesticLatest.high),
          low: Math.min(last.low, domesticLatest.low),
          close: domesticLatest.price,
        },
      ];
    }
    return [
      ...bars,
      {
        date: domesticLatest.date,
        open: domesticLatest.open,
        high: domesticLatest.high,
        low: domesticLatest.low,
        close: domesticLatest.price,
      },
    ];
  }, [bars, domesticLatest]);
  const dailyBars = useMemo(() => withChange(aggregateBars(liveBars, 'day')), [liveBars]);
  const intlPeriodBars = useMemo(
    () => withChange(aggregateBars(intlBars, filter.granularity)),
    [intlBars, filter.granularity]
  );
  const intlRangeBars = useMemo(
    () => filterByRange(intlPeriodBars, filter.range),
    [intlPeriodBars, filter.range]
  );
  const periodBars = useMemo(
    () => withChange(aggregateBars(liveBars, filter.granularity)),
    [liveBars, filter.granularity]
  );
  const rangeBars = useMemo(() => filterByRange(periodBars, filter.range), [periodBars, filter.range]);
  const resultBars = useMemo(
    () => filterByChange(rangeBars, filter.threshold, filter.direction),
    [rangeBars, filter.threshold, filter.direction]
  );

  const totalPages = Math.max(1, Math.ceil(resultBars.length / PAGE_SIZE));
  const currentPage = Math.min(page, totalPages);
  const pagedBars = useMemo(() => {
    const start = (currentPage - 1) * PAGE_SIZE;
    return resultBars.slice().reverse().slice(start, start + PAGE_SIZE);
  }, [resultBars, currentPage]);

  const domesticPrice = domesticLatest
    ? domesticLatest.price
    : dailyBars.length
      ? dailyBars[dailyBars.length - 1].close
      : null;

  // 最新价对比的基准：当天那根未收盘 K 线要用前一根收盘价
  const referenceClose = useMemo(() => {
    if (!dailyBars.length) return null;
    const last = dailyBars[dailyBars.length - 1];
    if (last.date === localDateKey() && dailyBars.length > 1) {
      return dailyBars[dailyBars.length - 2].close;
    }
    return last.close;
  }, [dailyBars]);

  const domesticChange =
    domesticPrice !== null && referenceClose
      ? ((domesticPrice - referenceClose) / referenceClose) * 100
      : null;

  useEffect(() => {
    if (!chartElement.current) return;
    if (!chartInstance.current) chartInstance.current = echarts.init(chartElement.current);

    const dates = rangeBars.map((bar) => barLabel(bar));
    const closes = rangeBars.map((bar) => ({
      value: Number(bar.close.toFixed(2)),
      changePercent: bar.changePercent,
    }));
    const highlights = resultBars.map((bar) => ({
      value: [barLabel(bar), Number(bar.close.toFixed(2))],
      changePercent: bar.changePercent,
    }));

    chartInstance.current.setOption(
      {
        backgroundColor: 'transparent',
        grid: { top: 26, right: 46, bottom: 34, left: 66 },
        LargestContentfulPaint: true,
        tooltip: {
          trigger: 'axis',
          backgroundColor: 'rgba(255, 253, 248, 0.97)',
          borderColor: 'rgba(168, 121, 31, 0.35)',
          borderWidth: 1,
          padding: [10, 12],
          textStyle: { color: '#1f1b14', fontSize: 12 },
          extraCssText: 'box-shadow: 0 14px 30px -18px rgba(56, 42, 18, 0.55);',
          axisPointer: { lineStyle: { color: 'rgba(168, 121, 31, 0.35)' } },
          formatter: (params) => {
            const list = Array.isArray(params) ? params : [params];
            const line = list.find((item) => item.seriesName === '收盘价');
            if (!line) return '';
            const change = line.data ? line.data.changePercent : null;
            const color = change > 0 ? '#c1462f' : change < 0 ? '#1f7a5a' : '#938a79';
            const rows = [
              `<div style="color:#938a79;font-size:11px">${line.axisValue}</div>`,
              `<div style="color:#a8791f;font-size:16px;margin-top:4px">¥${formatPrice(line.data.value)} 元/克</div>`,
            ];
            if (change !== null && change !== undefined) {
              rows.push(`<div style="color:${color};font-size:12px">涨跌 ${formatPercent(change)}</div>`);
            }
            return rows.join('');
          },
        },
        xAxis: {
          type: 'category',
          data: dates,
          boundaryGap: false,
          axisLine: { lineStyle: { color: 'rgba(31, 27, 20, 0.18)' } },
          axisTick: { show: false },
          axisLabel: { color: 'rgba(58, 51, 41, 0.72)', fontSize: 11 },
        },
        yAxis: {
          type: 'value',
          scale: true,
          axisLabel: {
            color: 'rgba(58, 51, 41, 0.72)',
            fontSize: 11,
            formatter: (value) => `${value}`,
          },
          splitLine: { lineStyle: { color: 'rgba(31, 27, 20, 0.08)' } },
        },
        series: [
          {
            name: '收盘价',
            type: 'line',
            data: closes,
            smooth: true,
            symbol: 'none',
            lineStyle: { width: 2, color: '#b5852a' },
            areaStyle: {
              color: new echarts.graphic.LinearGradient(0, 0, 0, 1, [
                { offset: 0, color: 'rgba(199, 154, 52, 0.26)' },
                { offset: 1, color: 'rgba(199, 154, 52, 0.02)' },
              ]),
            },
          },
          {
            name: '筛选命中',
            type: 'scatter',
            data: highlights,
            symbolSize: 6,
            itemStyle: {
              color: '#1f7a5a',
              borderColor: '#fffdf8',
              borderWidth: 1,
            },
            zlevel: 2,
          },
        ],
      },
      true
    );
  }, [rangeBars, resultBars]);

  useEffect(() => {
    if (!intlChartElement.current) return;
    if (!intlChartInstance.current) intlChartInstance.current = echarts.init(intlChartElement.current);

    const dates = intlRangeBars.map((bar) => barLabel(bar));
    const closes = intlRangeBars.map((bar) => ({
      value: Number(bar.close.toFixed(2)),
      changePercent: bar.changePercent,
    }));

    intlChartInstance.current.setOption(
      {
        backgroundColor: 'transparent',
        grid: { top: 26, right: 46, bottom: 34, left: 66 },
        tooltip: {
          trigger: 'axis',
          backgroundColor: 'rgba(255, 253, 248, 0.97)',
          borderColor: 'rgba(168, 121, 31, 0.35)',
          borderWidth: 1,
          padding: [10, 12],
          textStyle: { color: '#1f1b14', fontSize: 12 },
          extraCssText: 'box-shadow: 0 14px 30px -18px rgba(56, 42, 18, 0.55);',
          axisPointer: { lineStyle: { color: 'rgba(168, 121, 31, 0.35)' } },
          formatter: (params) => {
            const list = Array.isArray(params) ? params : [params];
            const line = list.find((item) => item.seriesName === '收盘价');
            if (!line) return '';
            const change = line.data ? line.data.changePercent : null;
            const color = change > 0 ? '#c1462f' : change < 0 ? '#1f7a5a' : '#938a79';
            const rows = [
              `<div style="color:#938a79;font-size:11px">${line.axisValue}</div>`,
              `<div style="color:#3a3329;font-size:16px;margin-top:4px">$${formatPrice(line.data.value)} 美元/盎司</div>`,
            ];
            if (change !== null && change !== undefined) {
              rows.push(`<div style="color:${color};font-size:12px">涨跌 ${formatPercent(change)}</div>`);
            }
            return rows.join('');
          },
        },
        xAxis: {
          type: 'category',
          data: dates,
          boundaryGap: false,
          axisLine: { lineStyle: { color: 'rgba(31, 27, 20, 0.18)' } },
          axisTick: { show: false },
          axisLabel: { color: 'rgba(58, 51, 41, 0.72)', fontSize: 11 },
        },
        yAxis: {
          type: 'value',
          scale: true,
          axisLabel: {
            color: 'rgba(58, 51, 41, 0.72)',
            fontSize: 11,
            formatter: (value) => formatPrice(value),
          },
          splitLine: { lineStyle: { color: 'rgba(31, 27, 20, 0.08)' } },
        },
        series: [
          {
            name: '收盘价',
            type: 'line',
            data: closes,
            smooth: true,
            symbol: 'none',
            lineStyle: { width: 2, color: '#3a3329' },
            areaStyle: {
              color: new echarts.graphic.LinearGradient(0, 0, 0, 1, [
                { offset: 0, color: 'rgba(58, 51, 41, 0.2)' },
                { offset: 1, color: 'rgba(58, 51, 41, 0.02)' },
              ]),
            },
          },
        ],
      },
      true
    );
  }, [intlRangeBars]);

  useEffect(() => {
    const element = chartElement.current;
    if (!element) return undefined;
    const observer = new ResizeObserver(() => {
      if (chartInstance.current) chartInstance.current.resize();
    });
    observer.observe(element);
    return () => observer.disconnect();
  }, []);

  useEffect(() => {
    const element = intlChartElement.current;
    if (!element) return undefined;
    const observer = new ResizeObserver(() => {
      if (intlChartInstance.current) intlChartInstance.current.resize();
    });
    observer.observe(element);
    return () => observer.disconnect();
  }, []);

  useEffect(
    () => () => {
      if (chartInstance.current) {
        chartInstance.current.dispose();
        chartInstance.current = null;
      }
      if (intlChartInstance.current) {
        intlChartInstance.current.dispose();
        intlChartInstance.current = null;
      }
    },
    []
  );

  const updateFilter = (patch) => setFilter((prev) => ({ ...prev, ...patch }));
  const resetFilter = () => setFilter({ ...DEFAULT_FILTER });

  const retry = () => {
    loadIntl();
    loadIntlBars();
    loadDomesticBars();
    loadDomesticLatest();
  };

  const handleExport = () => {
    const csv = `\uFEFF${toCsv(resultBars)}`;
    const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = `gold-au0-${filter.range}-${Date.now()}.csv`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  };

  const changeClass =
    domesticChange === null ? '' : domesticChange > 0 ? 'is-up' : domesticChange < 0 ? 'is-down' : 'is-flat';

  return (
    <div className="gold-price">
      <header className="gp-header">
        <div className="gp-header__main">
          <p className="gp-header__kicker">Gold Price</p>
          <h1 className="gp-header__title">金价行情</h1>
          <p className="gp-header__desc">
            国际伦敦金与国内沪金走势对照 · 筛选基于国内沪金 AU0（元/克）
          </p>
        </div>
        <div className="gp-header__meta">
          <span className="gp-status-dot" />
          <span>国内行情时间 {domesticLatest ? domesticLatest.at : '—'}</span>
        </div>
      </header>

      {status === 'error' ? (
        <section className="gp-error">
          <strong>国内行情加载失败</strong>
          <p>{domesticError || '请稍后重试'}</p>
          <button type="button" className="gp-btn gp-btn--primary" onClick={retry}>
            重试
          </button>
        </section>
      ) : (
        <>
          <section className="gp-quote">
            <div className="gp-quote__block">
              <span className="gp-quote__label">国际金价 · XAU/USD · 美元/盎司</span>
              <strong className="gp-quote__price">
                {intlQuote ? `$${formatPrice(intlQuote.price)}` : '—'}
              </strong>
              <span className="gp-quote__meta">更新于 {formatUpdatedAt(intlQuote?.updatedAt)}</span>
            </div>

            <div className="gp-quote__block">
              <span className="gp-quote__label">国内金价 · 沪金 AU0 · 元/克</span>
              <strong className="gp-quote__price gp-quote__price--domestic">
                {domesticPrice === null ? '—' : `¥${formatPrice(domesticPrice)}`}
              </strong>
              <span className={`gp-quote__change ${changeClass}`}>
                {formatPercent(domesticChange)}
                <em>较上一交易日</em>
              </span>
            </div>

            <p className="gp-quote__note">
              国际价格单位为美元/盎司，国内为人民币/克，两者口径不同，仅作参照、不可直接相减。
            </p>
          </section>

          {(intlError || intlBarsError || domesticError) && status === 'ready' && (
            <div className="gp-banner gp-banner--warn">
              {intlError && <div>国际报价刷新失败：{intlError}</div>}
              {intlBarsError && <div>国际历史行情获取失败：{intlBarsError}（当前展示缓存数据）</div>}
              {domesticError && <div>国内行情获取失败：{domesticError}（当前展示缓存或上一次成功的数据）</div>}
            </div>
          )}

          <section className="gp-body">
            <aside className="gp-filter">
              <div className="gp-filter__head">
                <h2>筛选条件</h2>
                <button type="button" className="gp-link" onClick={resetFilter}>
                  重置
                </button>
              </div>

              <label className="gp-field">
                <span>时间范围</span>
                <select
                  value={filter.range}
                  onChange={(event) => updateFilter({ range: event.target.value })}
                >
                  {RANGE_OPTIONS.map((option) => (
                    <option key={option.key} value={option.key}>
                      {option.label}
                    </option>
                  ))}
                </select>
              </label>

              <div className="gp-field">
                <span>粒度</span>
                <div className="gp-segmented">
                  {GRANULARITY_OPTIONS.map((option) => (
                    <button
                      key={option.key}
                      type="button"
                      className={filter.granularity === option.key ? 'is-active' : ''}
                      onClick={() => updateFilter({ granularity: option.key })}
                    >
                      {option.label}
                    </button>
                  ))}
                </div>
              </div>

              <p className="gp-filter__group-label">仅筛选国内行情</p>

              <label className="gp-field">
                <span>涨跌幅阈值 ≥</span>
                <div className="gp-threshold">
                  <input
                    type="number"
                    min="0"
                    step="0.1"
                    value={filter.threshold}
                    placeholder="不筛选"
                    onChange={(event) => updateFilter({ threshold: event.target.value })}
                  />
                  <em>%</em>
                </div>
              </label>

              <div className="gp-field">
                <span>方向</span>
                <div className="gp-segmented">
                  {DIRECTION_OPTIONS.map((option) => (
                    <button
                      key={option.key}
                      type="button"
                      className={filter.direction === option.key ? 'is-active' : ''}
                      onClick={() => updateFilter({ direction: option.key })}
                    >
                      {option.label}
                    </button>
                  ))}
                </div>
              </div>

              <p className="gp-filter__hint">
                阈值为空表示不筛选；涨跌幅按所选粒度相对上一根 K 线收盘价计算，仅作用于国内行情。
              </p>
            </aside>

            <div className="gp-content">
              <div className="gp-charts">
                <article className="gp-card">
                  <div className="gp-card__head">
                    <div>
                      <h2>国内金价走势</h2>
                      <p>沪金 AU0 日线，含今日实时行情 · 青色圆点为筛选命中点</p>
                    </div>
                    <span className="gp-card__count">{rangeBars.length} 个周期</span>
                  </div>
                  <div className="gp-chart-box">
                    <div className="gp-chart" ref={chartElement} />
                    {rangeBars.length === 0 && (
                      <p className="gp-chart-empty">暂无国内行情数据，请稍后重试或检查网络。</p>
                    )}
                  </div>
                </article>

                <article className="gp-card">
                  <div className="gp-card__head">
                    <div>
                      <h2>国际金价走势</h2>
                      <p>伦敦金 XAU 日线，单位美元/盎司 · 与左侧筛选共用时间范围与粒度</p>
                    </div>
                    <span className="gp-card__count">{intlRangeBars.length} 个周期</span>
                  </div>
                  <div className="gp-chart-box">
                    <div className="gp-chart" ref={intlChartElement} />
                    {intlRangeBars.length === 0 && (
                      <p className="gp-chart-empty">
                        {intlBarsStatus === 'loading'
                          ? '国际金价历史数据加载中…'
                          : intlBarsError
                            ? '国际金价历史数据加载失败，请稍后重试。'
                            : '暂无国际金价历史数据，请稍后重试或检查网络。'}
                      </p>
                    )}
                  </div>
                </article>
              </div>

              <article className="gp-card">
                <div className="gp-card__head">
                  <div>
                    <h2>筛选结果</h2>
                    <p>国内金价 · 共 {resultBars.length} 条记录</p>
                  </div>
                  <button
                    type="button"
                    className="gp-btn gp-btn--ghost"
                    onClick={handleExport}
                    disabled={!resultBars.length}
                  >
                    导出 CSV
                  </button>
                </div>

                {resultBars.length === 0 ? (
                  <div className="gp-empty">
                    <p>当前条件下没有匹配的行情记录。</p>
                    <p className="gp-empty__hint">试试降低涨跌幅阈值，或扩大时间范围。</p>
                    <button type="button" className="gp-btn gp-btn--primary" onClick={resetFilter}>
                      清空条件
                    </button>
                  </div>
                ) : (
                  <>
                    <div className="gp-table-wrap">
                      <table className="gp-table">
                        <thead>
                          <tr>
                            <th>周期</th>
                            <th>开盘</th>
                            <th>最高</th>
                            <th>最低</th>
                            <th>收盘</th>
                            <th>涨跌幅</th>
                          </tr>
                        </thead>
                        <tbody>
                          {pagedBars.map((bar) => (
                            <tr key={bar.date}>
                              <td>{barLabel(bar)}</td>
                              <td>{formatPrice(bar.open)}</td>
                              <td>{formatPrice(bar.high)}</td>
                              <td>{formatPrice(bar.low)}</td>
                              <td>{formatPrice(bar.close)}</td>
                              <td
                                className={
                                  bar.changePercent > 0
                                    ? 'is-up'
                                    : bar.changePercent < 0
                                      ? 'is-down'
                                      : ''
                                }
                              >
                                {formatPercent(bar.changePercent)}
                              </td>
                            </tr>
                          ))}
                        </tbody>
                      </table>
                    </div>

                    <div className="gp-pager">
                      <span className="gp-pager__info">
                        第 {(currentPage - 1) * PAGE_SIZE + 1}–
                        {Math.min(currentPage * PAGE_SIZE, resultBars.length)} 条 / 共 {resultBars.length} 条
                      </span>
                      <div className="gp-pager__actions">
                        <button
                          type="button"
                          className="gp-btn gp-btn--ghost"
                          onClick={() => setPage(currentPage - 1)}
                          disabled={currentPage <= 1}
                        >
                          上一页
                        </button>
                        <span className="gp-pager__page">
                          {currentPage} / {totalPages}
                        </span>
                        <button
                          type="button"
                          className="gp-btn gp-btn--ghost"
                          onClick={() => setPage(currentPage + 1)}
                          disabled={currentPage >= totalPages}
                        >
                          下一页
                        </button>
                      </div>
                    </div>
                  </>
                )}
              </article>
            </div>
          </section>

          <footer className="gp-footer">
            <p>
              国际报价来源：gold-api.com · 国际历史行情来源：新浪财经（伦敦金 XAU）· 国内行情来源：新浪财经（沪金
              AU0 期货，元/克）
            </p>
            <p>数据仅供参考，不构成任何投资建议。</p>
          </footer>
        </>
      )}
    </div>
  );
}
