import { useState, useRef, useCallback } from 'react';
import './index.css';

const formatSize = (bytes) => {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / (1024 * 1024)).toFixed(2)} MB`;
};

export default function ImageCompress() {
  const [originalFile, setOriginalFile] = useState(null);
  const [originalUrl, setOriginalUrl] = useState('');
  const [compressedUrl, setCompressedUrl] = useState('');
  const [compressedSize, setCompressedSize] = useState(0);
  const [quality, setQuality] = useState(80);
  const [isDragging, setIsDragging] = useState(false);
  const [isCompressing, setIsCompressing] = useState(false);
  const canvasRef = useRef(null);
  const fileInputRef = useRef(null);
  const compressedRef = useRef(null);

  const compressImage = useCallback((file, qual) => {
    return new Promise((resolve) => {
      const img = new Image();
      img.onload = () => {
        const canvas = canvasRef.current;
        if (!canvas) return resolve(null);
        canvas.width = img.naturalWidth;
        canvas.height = img.naturalHeight;
        const ctx = canvas.getContext('2d');
        ctx.drawImage(img, 0, 0);
        canvas.toBlob(
          (blob) => {
            if (blob) {
              const url = URL.createObjectURL(blob);
              resolve({ url, size: blob.size });
            } else {
              resolve(null);
            }
          },
          'image/jpeg',
          qual / 100
        );
      };
      img.src = URL.createObjectURL(file);
    });
  }, []);

  const handleFile = useCallback(async (file) => {
    if (!file || !file.type.startsWith('image/')) return;
    URL.revokeObjectURL(originalUrl);
    URL.revokeObjectURL(compressedUrl);
    setOriginalFile(file);
    setOriginalUrl(URL.createObjectURL(file));
    setCompressedUrl('');
    setCompressedSize(0);
    const result = await compressImage(file, quality);
    if (result) {
      setCompressedUrl(result.url);
      setCompressedSize(result.size);
    }
  }, [originalUrl, compressedUrl, quality, compressImage]);

  const handleQualityChange = async (e) => {
    const newQuality = Number(e.target.value);
    setQuality(newQuality);
    if (originalFile) {
      setIsCompressing(true);
      URL.revokeObjectURL(compressedUrl);
      const result = await compressImage(originalFile, newQuality);
      if (result) {
        setCompressedUrl(result.url);
        setCompressedSize(result.size);
      }
      setIsCompressing(false);
    }
  };

  const handleDrop = useCallback((e) => {
    e.preventDefault();
    setIsDragging(false);
    const file = e.dataTransfer.files[0];
    if (file) handleFile(file);
  }, [handleFile]);

  const handleDragOver = useCallback((e) => {
    e.preventDefault();
    setIsDragging(true);
  }, []);

  const handleDragLeave = useCallback(() => {
    setIsDragging(false);
  }, []);

  const handleDownload = () => {
    if (!compressedUrl) return;
    const a = document.createElement('a');
    a.href = compressedUrl;
    a.download = `compressed_${originalFile?.name || 'image.jpg'}`;
    a.click();
  };

  const resetAll = () => {
    URL.revokeObjectURL(originalUrl);
    URL.revokeObjectURL(compressedUrl);
    setOriginalFile(null);
    setOriginalUrl('');
    setCompressedUrl('');
    setCompressedSize(0);
    if (fileInputRef.current) fileInputRef.current.value = '';
  };

  const reduction = originalFile && compressedSize
    ? ((1 - compressedSize / originalFile.size) * 100).toFixed(1)
    : 0;

  return (
    <div className="img-compress">
      {/* 上传区 */}
      {!originalFile ? (
        <div
          className={`img-compress__upload ${isDragging ? 'dragging' : ''}`}
          onDrop={handleDrop}
          onDragOver={handleDragOver}
          onDragLeave={handleDragLeave}
          onClick={() => fileInputRef.current?.click()}
        >
          <input
            ref={fileInputRef}
            type="file"
            accept="image/*"
            className="img-compress__file-input"
            onChange={(e) => e.target.files[0] && handleFile(e.target.files[0])}
          />
          <div className="img-compress__upload-icon">
            <svg width="56" height="56" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
              <rect x="3" y="3" width="18" height="18" rx="2" />
              <circle cx="8.5" cy="8.5" r="1.5" />
              <path d="M21 15l-5-5L5 21" />
            </svg>
          </div>
          <p className="img-compress__upload-text">
            <strong>点击上传</strong> 或拖拽图片到这里
          </p>
          <p className="img-compress__upload-hint">支持 JPG、PNG、WebP、GIF 等常见图片格式</p>
        </div>
      ) : (
        <div className="img-compress__workspace">
          {/* 预览区 */}
          <div className="img-compress__preview-area">
            <div className="img-compress__preview-card">
              <div className="img-compress__preview-header">
                <span className="img-compress__preview-label">原图</span>
                <span className="img-compress__preview-size">{formatSize(originalFile.size)}</span>
              </div>
              <div className="img-compress__preview-img-wrap">
                {originalUrl && <img src={originalUrl} alt="Original" className="img-compress__preview-img" />}
              </div>
            </div>

            <div className="img-compress__arrow">
              <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M5 12h14M12 5l7 7-7 7" />
              </svg>
            </div>

            <div className="img-compress__preview-card img-compress__preview-card--compressed">
              <div className="img-compress__preview-header">
                <span className="img-compress__preview-label">压缩后</span>
                {compressedSize > 0 && (
                  <span className="img-compress__preview-size">
                    {formatSize(compressedSize)}
                    <span className="img-compress__reduction">-{reduction}%</span>
                  </span>
                )}
              </div>
              <div className="img-compress__preview-img-wrap">
                {compressedUrl ? (
                  <img ref={compressedRef} src={compressedUrl} alt="Compressed" className="img-compress__preview-img" />
                ) : (
                  <div className="img-compress__preview-placeholder">等待压缩...</div>
                )}
              </div>
            </div>
          </div>

          {/* 控制区 */}
          <div className="img-compress__controls">
            <div className="img-compress__quality">
              <div className="img-compress__quality-header">
                <label className="img-compress__quality-label">压缩质量</label>
                <span className="img-compress__quality-value">{quality}%</span>
              </div>
              <div className="img-compress__slider-wrap">
                <input
                  type="range"
                  min="1"
                  max="100"
                  value={quality}
                  onChange={handleQualityChange}
                  className="img-compress__slider"
                />
                <div className="img-compress__slider-track" style={{ width: `${quality}%` }} />
              </div>
              <div className="img-compress__quality-hints">
                <span onClick={() => { setQuality(30); compressImage(originalFile, 30).then(r => { if (r) { URL.revokeObjectURL(compressedUrl); setCompressedUrl(r.url); setCompressedSize(r.size); } }); }}>低压缩</span>
                <span onClick={() => { setQuality(70); compressImage(originalFile, 70).then(r => { if (r) { URL.revokeObjectURL(compressedUrl); setCompressedUrl(r.url); setCompressedSize(r.size); } }); }}>中等</span>
                <span onClick={() => { setQuality(100); compressImage(originalFile, 100).then(r => { if (r) { URL.revokeObjectURL(compressedUrl); setCompressedUrl(r.url); setCompressedSize(r.size); } }); }}>高质量</span>
              </div>
            </div>

            <div className="img-compress__actions">
              <button className="img-compress__btn img-compress__btn--ghost" onClick={resetAll}>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M3 12a9 9 0 1 0 9-9 9.75 9.75 0 0 0-6.74 2.74L3 8" />
                  <path d="M3 3v5h5" />
                </svg>
                重新上传
              </button>
              <button
                className="img-compress__btn img-compress__btn--primary"
                onClick={handleDownload}
                disabled={!compressedUrl || isCompressing}
              >
                {isCompressing ? (
                  <>
                    <span className="img-compress__spinner" />
                    压缩中...
                  </>
                ) : (
                  <>
                    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                      <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4" />
                      <polyline points="7,10 12,15 17,10" />
                      <line x1="12" y1="15" x2="12" y2="3" />
                    </svg>
                    下载图片
                  </>
                )}
              </button>
            </div>
          </div>

          {/* 统计 */}
          {compressedSize > 0 && (
            <div className="img-compress__stats">
              <div className="img-compress__stat-item">
                <span className="img-compress__stat-label">原文件大小</span>
                <span className="img-compress__stat-value">{formatSize(originalFile.size)}</span>
              </div>
              <div className="img-compress__stat-divider" />
              <div className="img-compress__stat-item">
                <span className="img-compress__stat-label">压缩后</span>
                <span className="img-compress__stat-value img-compress__stat-value--accent">{formatSize(compressedSize)}</span>
              </div>
              <div className="img-compress__stat-divider" />
              <div className="img-compress__stat-item">
                <span className="img-compress__stat-label">节省空间</span>
                <span className="img-compress__stat-value img-compress__stat-value--green">-{reduction}%</span>
              </div>
            </div>
          )}
        </div>
      )}
      <canvas ref={canvasRef} style={{ display: 'none' }} />
    </div>
  );
}
