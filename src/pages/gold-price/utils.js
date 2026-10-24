export const RANGE_OPTIONS = [
  { key: '1w', label: '近 1 周', days: 7 },
  { key: '1m', label: '近 1 月', days: 30 },
  { key: '3m', label: '近 3 月', days: 90 },
  { key: '6m', label: '近 6 月', days: 180 },
  { key: '1y', label: '近 1 年', days: 365 },
  { key: '3y', label: '近 3 年', days: 1095 },
  { key: '5y', label: '近 5 年', days: 1825 },
  { key: '10y', label: '近 10 年', days: 3650 },
  { key: '20y', label: '近 20 年', days: 7300 },
  { key: '30y', label: '近 30 年', days: 10950 },
  { key: 'all', label: '全部', days: Infinity },
];

export const GRANULARITY_OPTIONS = [
  { key: 'day', label: '日' },
  { key: 'week', label: '周' },
  { key: 'month', label: '月' },
  { key: 'year', label: '年' },
];

export const DIRECTION_OPTIONS = [
  { key: 'all', label: '不限' },
  { key: 'up', label: '仅上涨' },
  { key: 'down', label: '仅下跌' },
];

export const DEFAULT_FILTER = {
  range: '1m',
  granularity: 'day',
  threshold: '',
  direction: 'all',
};

function periodKey(dateStr, granularity) {
  if (granularity === 'year') return `${dateStr.slice(0, 4)}-01-01`;
  if (granularity === 'month') return `${dateStr.slice(0, 7)}-01`;
  if (granularity === 'week') {
    const date = new Date(`${dateStr}T00:00:00Z`);
    const offset = (date.getUTCDay() + 6) % 7; // 周一为一周起点
    date.setUTCDate(date.getUTCDate() - offset);
    return date.toISOString().slice(0, 10);
  }
  return dateStr;
}

function periodLabel(dateStr, granularity) {
  if (granularity === 'year') return dateStr.slice(0, 4);
  if (granularity === 'month') return dateStr.slice(0, 7);
  return dateStr;
}

// 把日线聚合成周线 / 月线 / 年线：开盘取首日、收盘取末日、高低取区间极值
export function aggregateBars(bars, granularity) {
  if (granularity === 'day') return bars.map((bar) => ({ ...bar, label: bar.date }));
  const map = new Map();
  bars.forEach((bar) => {
    const key = periodKey(bar.date, granularity);
    const current = map.get(key);
    if (!current) {
      map.set(key, {
        date: key,
        label: periodLabel(key, granularity),
        endDate: bar.date,
        open: bar.open,
        high: bar.high,
        low: bar.low,
        close: bar.close,
      });
      return;
    }
    current.high = Math.max(current.high, bar.high);
    current.low = Math.min(current.low, bar.low);
    current.close = bar.close;
    current.endDate = bar.date;
  });
  return [...map.values()].sort((a, b) => a.date.localeCompare(b.date));
}

// 涨跌幅相对上一根 K 线收盘价
export function withChange(bars) {
  return bars.map((bar, index) => {
    const prev = bars[index - 1];
    const changePercent = prev && prev.close ? ((bar.close - prev.close) / prev.close) * 100 : null;
    return { ...bar, changePercent };
  });
}

export function filterByRange(bars, rangeKey) {
  const option = RANGE_OPTIONS.find((item) => item.key === rangeKey) || RANGE_OPTIONS[1];
  if (!bars.length || option.days === Infinity) return bars;
  const last = new Date(`${bars[bars.length - 1].date}T00:00:00Z`);
  const start = new Date(last);
  start.setUTCDate(start.getUTCDate() - option.days + 1);
  return bars.filter((bar) => new Date(`${bar.date}T00:00:00Z`) >= start);
}

// threshold 为空字符串表示不筛选
export function filterByChange(bars, threshold, direction) {
  if (threshold === '' || threshold === null || threshold === undefined) return bars;
  const value = Number(threshold);
  if (!Number.isFinite(value) || value < 0) return bars;
  return bars.filter((bar) => {
    if (bar.changePercent === null) return false;
    if (Math.abs(bar.changePercent) < value) return false;
    if (direction === 'up') return bar.changePercent > 0;
    if (direction === 'down') return bar.changePercent < 0;
    return true;
  });
}

export function formatPrice(value) {
  return Number(value).toLocaleString('en-US', {
    minimumFractionDigits: 2,
    maximumFractionDigits: 2,
  });
}

export function formatPercent(value) {
  if (value === null || !Number.isFinite(value)) return '—';
  const sign = value > 0 ? '+' : '';
  return `${sign}${value.toFixed(2)}%`;
}

export function formatDate(dateStr) {
  return dateStr.length === 7 ? `${dateStr.slice(0, 4)}-${dateStr.slice(5, 7)}` : dateStr;
}

export function toCsv(bars) {
  const header = '日期,开盘,最高,最低,收盘,涨跌幅(%)';
  const lines = bars.map((bar) =>
    [
      bar.label || bar.date,
      bar.open.toFixed(2),
      bar.high.toFixed(2),
      bar.low.toFixed(2),
      bar.close.toFixed(2),
      bar.changePercent === null ? '' : bar.changePercent.toFixed(2),
    ].join(',')
  );
  return [header, ...lines].join('\n');
}
