import { createStore, combineReducers } from "redux";
import { calculate } from "./calculate";

// 全局你可以创建多个reducer 在这里统一在一起
const rootReducers = combineReducers({ calculate });
// 全局就管理一个store
export const store = createStore(rootReducers);
