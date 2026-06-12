import React, { useRef, useState, useEffect } from "react";
import "./index.scss";

// 视频元数据（可扩展）
const VIDEO_META = {
  "1.mp4": { title: "特效展示", category: "特效", desc: "俯瞰都市天际线的壮丽景色" },
  "2.mp4": { title: "场景推演", category: "推演", desc: "真实的仿真推演" },
  "3.mp4": { title: "产品展示", category: "模型", desc: "模型移动细节展示" },
  "4.mp4": { title: "技术演示", category: "教程", desc: "前端动效与交互实现过程" },
  "5.mp4": { title: "创意短片", category: "创意", desc: "富有创意的真实轨迹打击" },
};

function VideoPage() {
  const importAll = (r) => {
    const items = [];
    r.keys().forEach((key) => {
      items.push({ name: key.replace("./", ""), src: r(key) });
    });
    return items;
  };

  const videos = importAll(require.context("../../videos", false, /\.mp4$/));
  const [activeFilter, setActiveFilter] = useState("全部");
  const [playingId, setPlayingId] = useState(null);

  // 收集所有分类
  const categories = ["全部", ...new Set(videos.map((v) => VIDEO_META[v.name]?.category).filter(Boolean))];

  const filtered = activeFilter === "全部"
    ? videos
    : videos.filter((v) => VIDEO_META[v.name]?.category === activeFilter);

  return (
    <div className="video-page">
      <section className="video-content">
        {/* 分类筛选 */}
        <div className="video-filters">
          {categories.map((cat) => (
            <button
              key={cat}
              className={`filter-btn ${activeFilter === cat ? "active" : ""}`}
              onClick={() => setActiveFilter(cat)}
            >
              {cat}
            </button>
          ))}
        </div>

        {/* 瀑布流网格 */}
        <div className="video-grid">
          {filtered.map((video, index) => {
            const meta = VIDEO_META[video.name] || {};
            // 瀑布流：交替使用不同高度的卡片
            const isLarge = index % 5 === 0;
            const isWide = index % 5 === 3;

            return (
              <div
                key={video.name}
                className={`video-card ${isLarge ? "card-large" : ""} ${isWide ? "card-wide" : ""}`}
              >
                <div className="video-thumb">
                  <VideoPlayer
                    src={video.src}
                    poster={null}
                    isPlaying={playingId === video.name}
                    onPlay={() => setPlayingId(video.name)}
                    onStop={() => setPlayingId(null)}
                  />
                  {meta.category && <span className="video-tag">{meta.category}</span>}
                </div>
                <div className="video-info">
                  <h3 className="video-name">{meta.title || video.name}</h3>
                  <p className="video-desc">{meta.desc || ""}</p>
                </div>
              </div>
            );
          })}
        </div>
      </section>
    </div>
  );
}

// 视频播放器组件：hover 显示播放按钮，点击播放
function VideoPlayer({ src, isPlaying, onPlay, onStop }) {
  const videoRef = useRef(null);

  useEffect(() => {
    if (isPlaying && videoRef.current) {
      videoRef.current.play().catch(() => {});
    }
  }, [isPlaying]);

  const handlePlay = () => {
    if (videoRef.current) {
      if (videoRef.current.paused) {
        videoRef.current.play().catch(() => {});
        onPlay();
      } else {
        videoRef.current.pause();
        onStop();
      }
    }
  };

  const handleEnded = () => {
    onStop();
  };

  return (
    <div className="video-player" onClick={handlePlay}>
      <video
        ref={videoRef}
        src={src}
        muted
        loop
        playsInline
        preload="metadata"
        onEnded={handleEnded}
      />
      {!isPlaying && (
        <div className="play-overlay">
          <div className="play-btn">
            <svg viewBox="0 0 24 24" fill="currentColor" width="28" height="28">
              <path d="M8 5v14l11-7z" />
            </svg>
          </div>
        </div>
      )}
    </div>
  );
}

export default VideoPage;
