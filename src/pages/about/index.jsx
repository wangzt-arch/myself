import React, { useState, useEffect, useRef } from 'react'
import Header from '../../components/Header';
import Loading from '../../components/Loading2';
import isPc from '../../utils'
import me from './pdf/myself.pdf'
import { usePdf } from '@mikecousins/react-pdf';
import './index.css'

function About() {
  const [page, setPage] = useState(1);
  const [scale, setScale] = useState(1);
  const canvasRef = useRef(null);
  const { pdfDocument } = usePdf({
    file: me,
    page,
    scale,
    canvasRef,
  });

  useEffect(() => {
    isPc() ? setScale(1.3) : setScale(0.75);
  }, []);

  const prevPage = () => page > 1 && setPage(p => p - 1);
  const nextPage = () => pdfDocument && page < pdfDocument.numPages && setPage(p => p + 1);
  const zoomIn = () => setScale(s => Math.min(s + 0.15, 2.5));
  const zoomOut = () => setScale(s => Math.max(s - 0.15, 0.5));

  return (
    <div className="about-reader">
      <Header />

      <div className="reader-wrapper">
        {/* 侧边信息栏 */}
        <aside className="reader-sidebar">
          <div className="sidebar-avatar">
            <div className="avatar-placeholder">🌰</div>
          </div>
          <h2 className="sidebar-name">举个栗子</h2>
          <p className="sidebar-role">Frontend Developer</p>


          <div className="sidebar-skills">
            <p className="skills-label">技术栈</p>
            <div className="skill-tags">
              <span className="skill-tag">React</span>
              <span className="skill-tag">Vue</span>
              <span className="skill-tag">Three.js</span>
              <span className="skill-tag">TypeScript</span>
              <span className="skill-tag">Node.js</span>
              <span className="skill-tag">Cesium</span>
            </div>
          </div>
        </aside>

        {/* 主阅读区 */}
        <main className="reader-main">
          {/* 工具栏 */}
          <div className="reader-toolbar">
            <div className="toolbar-left">
              <span className="toolbar-filename">myself.pdf</span>
              <span className="toolbar-sep">·</span>
              <span className="toolbar-page">
                {pdfDocument ? `${page} / ${pdfDocument.numPages}` : '加载中...'}
              </span>
            </div>
            <div className="toolbar-right">
              <button
                className="toolbar-btn"
                onClick={zoomOut}
                title="缩小"
              >
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                  <path d="M3 8h10" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
                </svg>
              </button>
              <span className="toolbar-zoom">{Math.round(scale * 100)}%</span>
              <button
                className="toolbar-btn"
                onClick={zoomIn}
                title="放大"
              >
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                  <path d="M8 3v10M3 8h10" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
                </svg>
              </button>
              <span className="toolbar-divider" />
              <button
                className="toolbar-btn"
                onClick={prevPage}
                disabled={page <= 1}
                title="上一页"
              >
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                  <path d="M10 3L5 8l5 5" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </button>
              <button
                className="toolbar-btn"
                onClick={nextPage}
                disabled={pdfDocument ? page >= pdfDocument.numPages : true}
                title="下一页"
              >
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                  <path d="M6 3l5 5-5 5" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </button>
            </div>
          </div>

          {/* PDF 内容 */}
          <div className="reader-content">
            {!pdfDocument && (
              <div className="reader-loading">
                <Loading show={true} />
              </div>
            )}
            <canvas
              ref={canvasRef}
              style={{ opacity: pdfDocument ? 1 : 0, transition: 'opacity 0.4s ease' }}
            />
          </div>

          {/* 底部翻页 */}
          <div className="reader-footer">
            <button
              className="footer-btn"
              onClick={prevPage}
              disabled={page <= 1}
            >
              ← 上一页
            </button>
            <span className="footer-indicator">
              {pdfDocument ? `${page} / ${pdfDocument.numPages}` : '—'}
            </span>
            <button
              className="footer-btn"
              onClick={nextPage}
              disabled={pdfDocument ? page >= pdfDocument.numPages : true}
            >
              下一页 →
            </button>
          </div>
        </main>
      </div>
    </div>
  );
}

export default About;
