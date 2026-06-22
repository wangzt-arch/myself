/* 随机 JSON 生成工具 */

/* 基础数据类型生成 */
const randomString = () => {
  const len = Math.floor(Math.random() * 8) + 4;
  const chars = 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789';
  let result = '';
  for (let i = 0; i < len; i++) {
    result += chars[Math.floor(Math.random() * chars.length)];
  }
  return result;
};

const randomNumber = () => {
  const types = [
    () => Math.floor(Math.random() * 1000),
    () => Math.floor(Math.random() * 1000) / 100,
    () => -Math.floor(Math.random() * 1000),
    () => Math.random() * 10000,
  ];
  return types[Math.floor(Math.random() * types.length)]();
};

const randomBoolean = () => Math.random() > 0.5;

const randomDate = () => {
  const start = new Date(2000, 0, 1).getTime();
  const end = new Date(2030, 0, 1).getTime();
  return new Date(start + Math.random() * (end - start)).toISOString();
};

/* 键名模板（按业务分类） */
const keyTemplates = {
  user: ['id', 'name', 'email', 'age', 'avatar', 'phone', 'address', 'createdAt', 'isVip'],
  product: ['id', 'title', 'price', 'stock', 'category', 'tags', 'description', 'rating'],
  article: ['id', 'title', 'author', 'content', 'views', 'likes', 'publishedAt', 'tags'],
  order: ['orderId', 'userId', 'amount', 'status', 'items', 'createdAt', 'address'],
  generic: ['id', 'name', 'value', 'type', 'createdAt', 'updatedAt', 'status', 'description'],
};

const pickKeys = (count = 6) => {
  const pool = [...keyTemplates.generic, ...keyTemplates.user, ...keyTemplates.product];
  const shuffled = pool.sort(() => Math.random() - 0.5);
  return shuffled.slice(0, count);
};

/* 随机值（递归生成） */
const randomValue = (depth = 0) => {
  if (depth > 3) return randomString();

  const types = ['string', 'number', 'boolean', 'null', 'array', 'object'];
  const weights = [0.35, 0.25, 0.1, 0.05, 0.1, 0.15];
  let rand = Math.random();
  let selected = 'string';
  for (let i = 0; i < types.length; i++) {
    if (rand < weights[i]) { selected = types[i]; break; }
    rand -= weights[i];
  }

  switch (selected) {
    case 'string':
      return Math.random() < 0.3 ? randomDate() : randomString();
    case 'number':
      return randomNumber();
    case 'boolean':
      return randomBoolean();
    case 'null':
      return null;
    case 'array': {
      const len = Math.floor(Math.random() * 5) + 2;
      const arr = [];
      for (let i = 0; i < len; i++) arr.push(randomValue(depth + 1));
      return arr;
    }
    case 'object':
      return randomObject(depth + 1);
    default:
      return null;
  }
};

/* 随机对象 */
export const randomObject = (depth = 0) => {
  const obj = {};
  const keys = pickKeys(4 + Math.floor(Math.random() * 4));
  keys.forEach((k) => { obj[k] = randomValue(depth); });
  return obj;
};

/* 随机数组 */
export const randomArray = (size = 5) => {
  return Array.from({ length: size }, () => randomObject());
};

/* 模板生成器 */
const templates = {
  user: () => ({
    id: Math.floor(Math.random() * 100000),
    name: randomString(),
    email: `${randomString()}@example.com`,
    age: Math.floor(Math.random() * 50) + 18,
    phone: `1${Math.floor(Math.random() * 9) + 1}${Math.floor(Math.random() * 1000000000).toString().padStart(9, '0')}`,
    isVip: randomBoolean(),
    createdAt: randomDate(),
  }),
  product: () => ({
    id: Math.floor(Math.random() * 10000),
    title: randomString(),
    price: Math.floor(Math.random() * 10000) / 100,
    stock: Math.floor(Math.random() * 500),
    rating: Math.floor(Math.random() * 50) / 10,
    tags: ['新品', '推荐', '热销'].slice(0, Math.floor(Math.random() * 3) + 1),
    createdAt: randomDate(),
  }),
  article: () => ({
    id: Math.floor(Math.random() * 10000),
    title: randomString(),
    author: randomString(),
    views: Math.floor(Math.random() * 100000),
    likes: Math.floor(Math.random() * 10000),
    publishedAt: randomDate(),
    tags: [randomString(), randomString()],
  }),
  tree: () => {
    const build = (depth) => {
      if (depth > 3) return null;
      return {
        id: Math.floor(Math.random() * 10000),
        name: randomString(),
        children: [
          build(depth + 1),
          build(depth + 1),
        ].filter(Boolean),
      };
    };
    return build(0);
  },
};

/* 公开 API */
export const generateRandomJson = (template = 'random', size = 1) => {
  if (template === 'array') return randomArray(size);
  if (templates[template]) return templates[template]();
  return randomObject();
};

export const templatesList = [
  { id: 'random', label: '随机对象', desc: '随机生成嵌套对象' },
  { id: 'array', label: '随机数组', desc: '生成对象数组（5条）' },
  { id: 'user', label: '用户', desc: '用户信息模板' },
  { id: 'product', label: '商品', desc: '商品信息模板' },
  { id: 'article', label: '文章', desc: '文章信息模板' },
  { id: 'tree', label: '树形结构', desc: '递归嵌套数据' },
];