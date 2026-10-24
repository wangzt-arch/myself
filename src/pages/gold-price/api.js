const INTL_URL = 'https://api.gold-api.com/price/XAU';
const SINA_BASE = 'https://stock.finance.sina.com.cn/futures/api/jsonp.php';
// 新浪财经国际/国内期货 JSONP 接口，用 script 标签加载，不受跨域限制
const SINA_KLINE_URL = `${SINA_BASE}/var%20_AU0=/InnerFuturesNewService.getDailyKLine?symbol=AU0`;
const SINA_MINUTE_URL = `${SINA_BASE}/var%20_AU0M=/InnerFuturesNewService.getFewMinLine?symbol=AU0&type=1`;

const KEY_BARS = 'gold-price:domestic-bars:v3';
const KEY_BARS_AT = 'gold-price:domestic-bars-at:v3';
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

function loadJsonp(url, globalName, timeout = 20000) {
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
      if (!value) reject(new Error('国内行情数据为空'));
      else resolve(value);
    };
    script.onerror = () => {
      cleanup();
      reject(new Error('国内行情接口无法访问'));
    };
    timer = setTimeout(() => {
      cleanup();
      reject(new Error('国内行情接口请求超时'));
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

/* ---------- 国内金价（沪金 AU0，元/克） ---------- */

export async function fetchDomesticBars() {
  const rows = await loadJsonp(SINA_KLINE_URL, '_AU0');
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
  const rows = await loadJsonp(SINA_MINUTE_URL, '_AU0M');
  const valid = (Array.isArray(rows) ? rows : []).filter((row) => Number.isFinite(Number(row.c)));
  if (!valid.length) return null;
  const last = valid[valid.length - 1];
  return { price: Number(last.c), at: last.d };
}

/* ---------- 本地缓存与筛选条件 ---------- */

export function readCachedBars() {
  try {
    const raw = localStorage.getItem(KEY_BARS);
    const at = Number(localStorage.getItem(KEY_BARS_AT) || 0);
    const parsed = raw ? JSON.parse(raw) : [];
    return { bars: Array.isArray(parsed) ? parsed.filter(isBar) : [], at };
  } catch (e) {
    return { bars: [], at: 0 };
  }
}

export function saveCachedBars(bars) {
  try {
    localStorage.setItem(KEY_BARS, JSON.stringify(bars.slice(-MAX_BARS)));
    localStorage.setItem(KEY_BARS_AT, String(Date.now()));
  } catch (e) {
    /* 容量写满时静默失败 */
  }
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
