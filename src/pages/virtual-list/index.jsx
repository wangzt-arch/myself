import React, { useState, useEffect, useRef, useCallback, useMemo } from "react";
import SearchBar from "./SearchBar";
import Skeleton from "./Skeleton";
import GroupCollapse from "./GroupCollapse";
import "./index.scss";

// 图表仪表盘色系 — 与 Cesium 模块统一
const CATEGORY_STYLES = {
  "前端技术": { color: "#00c8ff", bg: "rgba(0, 200, 255, 0.12)", label: "FE" },
  "后端技术": { color: "#4ade80", bg: "rgba(74, 222, 128, 0.12)", label: "BE" },
  "DevOps": { color: "#8b5cff", bg: "rgba(139, 92, 255, 0.12)", label: "DO" },
  "数据库": { color: "#f59e0b", bg: "rgba(245, 158, 11, 0.12)", label: "DB" },
  "移动端": { color: "#f43f5e", bg: "rgba(244, 63, 94, 0.12)", label: "MB" },
};

const generateData = (count, startIndex = 0) => {
  const categories = Object.keys(CATEGORY_STYLES);
  const titleTemplates = [
    "深入理解 React 渲染机制",
    "VUE 开发入门指南",
    "Node.js 性能优化实战",
    "Docker 容器化部署指南",
    "MySQL 索引优化详解",
    "React Native 跨平台开发",
    "TypeScript 高级类型系统",
    "微服务架构设计模式",
    "Kubernetes 从入门到精通",
    "Redis 缓存策略最佳实践",
    "Flutter 动画开发技巧",
  ];
  const items = [];
  for (let i = 0; i < count; i++) {
    const index = startIndex + i;
    const category = categories[index % categories.length];
    items.push({
      id: index,
      title: titleTemplates[index % titleTemplates.length],
      index: index + 1,
      category,
      author: `作者 ${(index % 20) + 1}`,
      date: new Date(Date.now() - index * 86400000).toLocaleDateString("zh-CN"),
      views: Math.floor(Math.random() * 10000),
      likes: Math.floor(Math.random() * 500),
      readTime: Math.floor(Math.random() * 15) + 3,
      tags: [category, "技术", "教程"].slice(0, (index % 3) + 1),
    });
  }
  return items;
};

const VirtualList = ({ items, itemHeight = 92, height = 520, selectedIds, onToggleSelect, keyboardIndex, startIndex }) => {
  const containerRef = useRef(null);
  const [scrollTop, setScrollTop] = useState(0);

  const handleScroll = useCallback((e) => {
    setScrollTop(e.target.scrollTop);
  }, []);

  const visibleCount = Math.ceil(height / itemHeight) + 4;
  const startIdx = Math.max(0, Math.floor(scrollTop / itemHeight) - 2);
  const endIndex = Math.min(items.length, startIdx + visibleCount);
  const visibleItems = items.slice(startIdx, endIndex);
  const totalHeight = items.length * itemHeight;
  const offsetY = startIdx * itemHeight;

  return (
    <div
      ref={containerRef}
      className="vl-container"
      style={{ height }}
      onScroll={handleScroll}
    >
      <div className="vl-phantom" style={{ height: totalHeight }} />
      <div className="vl-content" style={{ transform: `translateY(${offsetY}px)` }}>
        {visibleItems.map((item, idx) => {
          const style = CATEGORY_STYLES[item.category] || CATEGORY_STYLES["前端技术"];
          const isSelected = selectedIds.has(item.id);
          const isFocused = keyboardIndex === startIdx + idx;
          return (
            <div
              key={item.id}
              className={`vl-item ${isSelected ? "selected" : ""} ${isFocused ? "focused" : ""}`}
              style={{ height: itemHeight, "--accent": style.color, "--accent-bg": style.bg }}
              onClick={() => onToggleSelect(item.id)}
            >
              <div className="vl-item-accent" />
              {<div className="vl-item-check" style={{ color: style.color }}> {isSelected&&'✓'}</div>}
              <div className="vl-item-index">{String(item.index).padStart(3, "0")}</div>
              <div className="vl-item-avatar">
                {item.author}
              </div>
              <div className="vl-item-main">
                <div className="vl-item-row-top">
                  <span className="vl-item-title">{item.title}</span>
                  <span className="vl-item-category" style={{ color: style.color, background: style.bg }}>
                    {style.label}
                  </span>
                </div>
                <div className="vl-item-meta">
                  <span className="vl-meta-item">
                    <span className="vl-dot" style={{ background: style.color }} />
                    {item.author}
                  </span>
                  <span className="vl-sep">·</span>
                  <span className="vl-meta-item">{item.date}</span>
                  <span className="vl-sep">·</span>
                  <span className="vl-meta-item">{item.readTime} 分钟</span>
                  <span className="vl-meta-right">
                    <span>👁 {item.views.toLocaleString()}</span>
                    <span>❤ {item.likes}</span>
                  </span>
                </div>
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
};

function VirtualListPage() {
  const [allData, setAllData] = useState(() => generateData(100));
  const [loading, setLoading] = useState(false);
  const [hasMore, setHasMore] = useState(true);
  const [searchTerm, setSearchTerm] = useState("");
  const [sortOrder, setSortOrder] = useState("desc");
  const [groupMode, setGroupMode] = useState(false);
  const [selectedIds, setSelectedIds] = useState(new Set());
  const [keyboardIndex, setKeyboardIndex] = useState(-1);
  const listRef = useRef(null);

  const filteredData = useMemo(() => {
    if (!searchTerm) return allData;
    const lower = searchTerm.toLowerCase();
    return allData.filter(
      (item) =>
        item.title.toLowerCase().includes(lower) ||
        item.author.toLowerCase().includes(lower) ||
        item.category.toLowerCase().includes(lower)
    );
  }, [allData, searchTerm]);

  const sortedData = useMemo(() => {
    const data = [...filteredData];
    if (sortOrder === "desc") {
      data.sort((a, b) => b.id - a.id);
    } else {
      data.sort((a, b) => a.id - b.id);
    }
    return data;
  }, [filteredData, sortOrder]);

  const groupedData = useMemo(() => {
    if (!groupMode) return null;
    const groups = {};
    sortedData.forEach((item) => {
      if (!groups[item.category]) groups[item.category] = [];
      groups[item.category].push(item);
    });
    return groups;
  }, [sortedData, groupMode]);

  const loadMore = useCallback(() => {
    if (loading || !hasMore) return;
    setLoading(true);
    setTimeout(() => {
      const newData = generateData(50, allData.length);
      setAllData((prev) => [...prev, ...newData]);
      setLoading(false);
      if (allData.length >= 500) setHasMore(false);
    }, 800);
  }, [loading, hasMore, allData.length]);

  useEffect(() => {
    const container = listRef.current;
    if (!container) return;
    const handleScroll = (e) => {
      const { scrollTop, scrollHeight, clientHeight } = e.target;
      if (scrollHeight - scrollTop - clientHeight < 120) loadMore();
    };
    container.addEventListener("scroll", handleScroll);
    return () => container.removeEventListener("scroll", handleScroll);
  }, [loadMore]);

  useEffect(() => {
    const handleKeyDown = (e) => {
      if (groupMode) return;
      const data = sortedData;
      if (e.key === "ArrowDown") {
        e.preventDefault();
        setKeyboardIndex((prev) => Math.min(prev + 1, data.length - 1));
      } else if (e.key === "ArrowUp") {
        e.preventDefault();
        setKeyboardIndex((prev) => Math.max(prev - 1, 0));
      } else if (e.key === "Enter" && keyboardIndex >= 0) {
        e.preventDefault();
        const item = data[keyboardIndex];
        // alert(`选中: ${item.title}`);
      } else if (e.key === "Escape") {
        setKeyboardIndex(-1);
        setSelectedIds(new Set());
      } else if (e.key === "a" && e.ctrlKey) {
        e.preventDefault();
        setSelectedIds(new Set(data.map((item) => item.id)));
      }
    };
    window.addEventListener("keydown", handleKeyDown);
    return () => window.removeEventListener("keydown", handleKeyDown);
  }, [sortedData, keyboardIndex, groupMode]);

  const toggleSelect = useCallback((id) => {
    setSelectedIds((prev) => {
      const next = new Set(prev);
      next.has(id) ? next.delete(id) : next.add(id);
      return next;
    });
  }, []);

  const selectAll = useCallback(() => {
    if (selectedIds.size === sortedData.length) setSelectedIds(new Set());
    else setSelectedIds(new Set(sortedData.map((item) => item.id)));
  }, [sortedData, selectedIds.size]);

  const deleteSelected = useCallback(() => {
    if (selectedIds.size === 0) return;
    setAllData((prev) => prev.filter((item) => !selectedIds.has(item.id)));
    setSelectedIds(new Set());
  }, [selectedIds]);

  const jumpToIndex = useCallback(
    (index) => {
      if (!listRef.current || index < 0 || index >= sortedData.length) return;
      listRef.current.scrollTop = index * 92;
      setKeyboardIndex(index);
    },
    [sortedData.length]
  );

  // 计算分类统计 — 用于顶部的分类统计环
  const categoryStats = useMemo(() => {
    const stats = {};
    allData.forEach((item) => {
      stats[item.category] = (stats[item.category] || 0) + 1;
    });
    return stats;
  }, [allData]);

  return (
    <div className="vl-page">
      {/* 页面标题区 */}
      <header className="vl-hero">
        <h1 className="vl-title">
          虚拟<span className="vl-title-accent">列表</span>
        </h1>
        <p className="vl-subtitle">
          面向海量数据的渲染引擎 — 仅保留可视窗口内的节点，滚动恒时复杂度
        </p>

        {/* 分类统计环 */}
        <div className="vl-stat-row">
          {Object.entries(categoryStats).map(([cat, count]) => {
            const style = CATEGORY_STYLES[cat];
            return (
              <div key={cat} className="vl-stat-chip" style={{ "--chip-color": style.color, "--chip-bg": style.bg }}>
                <span className="vl-chip-dot" />
                <span className="vl-chip-label">{cat}</span>
                <span className="vl-chip-count">{count}</span>
              </div>
            );
          })}
        </div>
      </header>

      {/* 工具栏 */}
      <div className="vl-toolbar">
        <div className="vl-toolbar-left">
          <SearchBar value={searchTerm} onChange={setSearchTerm} />
        </div>

        <div className="vl-toolbar-right">
          <select
            className="vl-select"
            value={sortOrder}
            onChange={(e) => setSortOrder(e.target.value)}
          >
            <option value="desc">最新优先</option>
            <option value="asc">最旧优先</option>
          </select>

          <button className={`vl-btn ${groupMode ? "active" : ""}`} onClick={() => setGroupMode(!groupMode)}>
            <span className="vl-btn-icon">§</span>
            {groupMode ? "列表视图" : "分组视图"}
          </button>

          <button className="vl-btn" onClick={selectAll}>
            <span className="vl-btn-icon">↯</span>
            {selectedIds.size === sortedData.length ? "取消全选" : "全选"}
          </button>

          {selectedIds.size > 0 && (
            <button className="vl-btn danger" onClick={deleteSelected}>
              <span className="vl-btn-icon">✕</span>
              删除 {selectedIds.size}
            </button>
          )}
        </div>
      </div>

      {/* 跳转 + 统计 */}
      <div className="vl-info-bar">
        <div className="vl-info-left">
          <span className="vl-info-count">{sortedData.length.toLocaleString()}</span>
          <span className="vl-info-label">条记录</span>
          {selectedIds.size > 0 && (
            <>
              <span className="vl-info-sep">/</span>
              <span className="vl-info-count selected">{selectedIds.size}</span>
              <span className="vl-info-label">已选中</span>
            </>
          )}
          {!hasMore && <span className="vl-info-tag">全部加载完毕</span>}
          {hasMore && loading && <span className="vl-info-tag loading">加载中 ·</span>}
        </div>
        <div className="vl-info-right">
          <span className="vl-jump-label">跳转到</span>
          <input
            type="number"
            className="vl-jump-input"
            placeholder="#"
            min={1}
            max={sortedData.length}
            onKeyDown={(e) => {
              if (e.key === "Enter") {
                jumpToIndex(parseInt(e.target.value) - 1);
                e.target.value = "";
              }
            }}
          />
          <span className="vl-kbd-hint">↑ ↓ 导航 · ↵ 确认</span>
        </div>
      </div>

      {/* 列表主体 */}
      {loading && allData.length === 0 ? (
        <Skeleton count={8} />
      ) : groupMode ? (
        <div className="vl-group-list" ref={listRef}>
          {Object.entries(groupedData || {}).map(([groupName, groupItems]) => {
            const style = CATEGORY_STYLES[groupName];
            return (
              <GroupCollapse
                key={groupName}
                title={groupName}
                count={groupItems.length}
                items={groupItems}
                selectedIds={selectedIds}
                onToggleSelect={toggleSelect}
                accentColor={style?.color}
                accentBg={style?.bg}
                accentLabel={style?.label}
              />
            );
          })}
          {loading && (
            <div className="vl-loading-more">
              <Skeleton count={3} />
            </div>
          )}
        </div>
      ) : (
        <div ref={listRef} className="vl-list-wrap">
          <VirtualList
            items={sortedData}
            itemHeight={92}
            height={540}
            selectedIds={selectedIds}
            onToggleSelect={toggleSelect}
            keyboardIndex={keyboardIndex}
          />
          {loading && allData.length > 0 && (
            <div className="vl-loading-more">
              <Skeleton count={3} />
            </div>
          )}
        </div>
      )}

      {/* 空状态 */}
      {sortedData.length === 0 && !loading && (
        <div className="vl-empty">
          <div className="vl-empty-orb" />
          <div className="vl-empty-title">无匹配结果</div>
          <div className="vl-empty-desc">当前搜索条件下没有符合的内容</div>
          <button className="vl-empty-btn" onClick={() => setSearchTerm("")}>
            重置搜索
          </button>
        </div>
      )}
    </div>
  );
}

export default VirtualListPage;
