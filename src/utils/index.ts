export const characterCount = (str, char) => str.split(char).length - 1;
/**
 * 判断是否为空
 * @param obj
 * @returns true|false
 */
export const isEmpty = (obj) =>
  Reflect.ownKeys(obj).length === 0 && obj.constructor === Object;
