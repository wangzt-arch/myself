const INTL_URL = 'https://api.gold-api.com/price/XAU';
const SINA_BASE = 'https://stock.finance.sina.com.cn/futures/api/jsonp.php';
// 新浪财经国际/国内期货 JSONP 接口，用 script 标签加载，不受跨域限制
const SINA_KLINE_URL = `${SINA_BASE}/var%20_AU0=/InnerFuturesNewService.getDailyKLine?symbol=AU0`;
const SINA_MINUTE_URL = `${SINA_BASE}/var%20_AU0M=/InnerFuturesNewService.getFewMinLine?symbol=AU0&type=1`;
const SINA_HOUR_URL = `${SINA_BASE}/var%20_AU0H=/InnerFuturesNewService.getFewMinLine?symbol=AU0&type=60`;
const SINA_XAU_URL = `${SINA_BASE}/var%20_XAU=/GlobalFuturesService.getGlobalFuturesDailyKLine?symbol=XAU`;

const KEY_BARS = 'gold-price:domestic-bars:v3';
const KEY_BARS_AT = 'gold-price:domestic-bars-at:v3';
const KEY_INTL_BARS = 'gold-price:intl-bars:v1';
const KEY_INTL_BARS_AT = 'gold-price:intl-bars-at:v1';
const KEY_FILTER = 'gold-price:filter:v1';

const MAX_BARS = 6000;
const HISTORY_TTL = 6 * 60 * 60 * 1000;

function isBar(bar) {
  return (
    bar &&
    typeof bar.date === 'string' &&
    [bar.open, bar.high, bar.low, bar.close].every((value) => Number.isFinite(Number(value)))
  );
}

function sortBars(bars) {
  return bars.sort((a, b) => a.date.localeCompare(b.date));
}

export function localDateKey(date = new Date()) {
  const y = date.getFullYear();
  const m = String(date.getMonth() + 1).padStart(2, '0');
  const d = String(date.getDate()).padStart(2, '0');
  return `${y}-${m}-${d}`;
}

/* ---------- JSONP 加载 ---------- */

function loadJsonp(url, globalName, label, timeout = 20000) {
  return new Promise((resolve, reject) => {
    let timer = null;
    const script = document.createElement('script');
    const cleanup = () => {
      if (timer) clearTimeout(timer);
      script.remove();
    };

    // var 声明的全局变量不可删除，只能重置值
    try {
      window[globalName] = undefined;
    } catch (e) {
      /* 忽略重置失败 */
    }
    script.async = true;
    script.src = url;
    script.onload = () => {
      const value = window[globalName];
      cleanup();
      if (!value) reject(new Error(`${label}数据为空`));
      else resolve(value);
    };
    script.onerror = () => {
      cleanup();
      reject(new Error(`${label}接口无法访问`));
    };
    timer = setTimeout(() => {
      cleanup();
      reject(new Error(`${label}接口请求超时`));
    }, timeout);

    document.head.appendChild(script);
  });
}

/* ---------- 国际实时报价 ---------- */

export async function fetchIntlQuote() {
  const res = await fetch(INTL_URL, { cache: 'no-store' });
  if (!res.ok) throw new Error(`国际报价请求失败（HTTP ${res.status}）`);
  const data = await res.json();
  const price = Number(data.price);
  if (!Number.isFinite(price)) throw new Error('国际报价数据格式异常');
  return {
    price,
    updatedAt: data.updatedAt || new Date().toISOString(),
    currency: data.currency || 'USD',
  };
}

/* ---------- 国际金价（伦敦金 XAU，美元/盎司） ---------- */

export async function fetchIntlBars() {
  const rows = await loadJsonp(SINA_XAU_URL, '_XAU', '国际历史行情');
  const bars = (Array.isArray(rows) ? rows : [])
    .map((row) => ({
      date: String(row.date).slice(0, 10),
      open: Number(row.open),
      high: Number(row.high),
      low: Number(row.low),
      close: Number(row.close),
    }))
    .filter(isBar);
  if (!bars.length) throw new Error('国际历史行情为空');
  return sortBars(bars).slice(-MAX_BARS);
}

/* ---------- 国内金价（沪金 AU0，元/克） ---------- */

export async function fetchDomesticBars() {
  const rows = await loadJsonp(SINA_KLINE_URL, '_AU0', '国内行情');
  const bars = (Array.isArray(rows) ? rows : [])
    .map((row) => ({
      date: String(row.d).slice(0, 10),
      open: Number(row.o),
      high: Number(row.h),
      low: Number(row.l),
      close: Number(row.c),
    }))
    .filter(isBar);
  if (!bars.length) throw new Error('国内历史行情为空');
  return sortBars(bars).slice(-MAX_BARS);
}

export async function fetchDomesticLatest() {
  const rows = await loadJsonp(SINA_MINUTE_URL, '_AU0M', '国内行情');
  const valid = (Array.isArray(rows) ? rows : []).filter((row) => Number.isFinite(Number(row.c)));
  if (!valid.length) return null;
  const last = valid[valid.length - 1];
  const latest = { price: Number(last.c), at: last.d };
  const todayBar = buildTodayBar(valid);
  return todayBar ? { ...latest, ...todayBar } : latest;
}

// 日线接口在收盘结算前不含当天，用分时数据聚合当日夜盘 + 日盘 K 线
function buildTodayBar(rows) {
  const now = new Date();
  if (now.getDay() === 0 || now.getDay() === 6) return null;

  const sessionStart = new Date(now);
  sessionStart.setHours(20, 0, 0, 0);
  sessionStart.setDate(sessionStart.getDate() - 1);
  while (sessionStart.getDay() === 0 || sessionStart.getDay() === 6) {
    sessionStart.setDate(sessionStart.getDate() - 1);
  }

  const sessionRows = rows.filter((row) => {
    const stamp = new Date(String(row.d).replace(' ', 'T'));
    return !Number.isNaN(stamp.getTime()) && stamp >= sessionStart && stamp <= now;
  });
  if (!sessionRows.length) return null;

  const toNumber = (value) => (Number.isFinite(Number(value)) ? Number(value) : null);
  const highs = sessionRows.map((row) => toNumber(row.h)).filter((value) => value !== null);
  const lows = sessionRows.map((row) => toNumber(row.l)).filter((value) => value !== null);
  const open = toNumber(sessionRows[0].o) ?? toNumber(sessionRows[0].c);
  const close = toNumber(sessionRows[sessionRows.length - 1].c);

  return {
    date: localDateKey(now),
    open: open ?? close,
    high: highs.length ? Math.max(...highs) : close,
    low: lows.length ? Math.min(...lows) : close,
  };
}

/* ---------- 国内 60 分钟线（沪金 AU0） ---------- */

export async function fetchDomesticHourBars() {
  const rows = await loadJsonp(SINA_HOUR_URL, '_AU0H', '国内 60 分钟行情');
  const bars = (Array.isArray(rows) ? rows : [])
    .map((row) => ({
      date: String(row.d).slice(0, 16),
      open: Number(row.o),
      high: Number(row.h),
      low: Number(row.l),
      close: Number(row.c),
    }))
    .filter(isBar);
  if (!bars.length) throw new Error('国内 60 分钟行情为空');
  return sortBars(bars);
}

/* ---------- 本地缓存与筛选条件 ---------- */

function readCache(key, atKey) {
  try {
    const raw = localStorage.getItem(key);
    const at = Number(localStorage.getItem(atKey) || 0);
    const parsed = raw ? JSON.parse(raw) : [];
    return { bars: Array.isArray(parsed) ? parsed.filter(isBar) : [], at };
  } catch (e) {
    return { bars: [], at: 0 };
  }
}

function saveCache(key, atKey, bars) {
  try {
    localStorage.setItem(key, JSON.stringify(bars.slice(-MAX_BARS)));
    localStorage.setItem(atKey, String(Date.now()));
  } catch (e) {
    /* 容量写满时静默失败 */
  }
}

export function readCachedBars() {
  return readCache(KEY_BARS, KEY_BARS_AT);
}

export function saveCachedBars(bars) {
  saveCache(KEY_BARS, KEY_BARS_AT, bars);
}

export function readCachedIntlBars() {
  return readCache(KEY_INTL_BARS, KEY_INTL_BARS_AT);
}

export function saveCachedIntlBars(bars) {
  saveCache(KEY_INTL_BARS, KEY_INTL_BARS_AT, bars);
}

export function isHistoryFresh(at) {
  return Date.now() - at < HISTORY_TTL;
}

export function readFilter() {
  try {
    const raw = localStorage.getItem(KEY_FILTER);
    return raw ? JSON.parse(raw) : null;
  } catch (e) {
    return null;
  }
}

export function saveFilter(filter) {
  try {
    localStorage.setItem(KEY_FILTER, JSON.stringify(filter));
  } catch (e) {
    /* 静默失败 */
  }
}
