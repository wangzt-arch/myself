const STORAGE_KEY = 'ai-chat-messages';
const META_KEY = 'ai-chat-meta';
const MAX_MESSAGES = 200;
const MAX_AGE_DAYS = 7;

export function saveMessages(messages) {
  try {
    // 只保存已完成的对话（isStreaming 状态不需要持久化）
    const persistable = messages
      .filter((msg) => !msg.isStreaming && msg.content)
      .slice(-MAX_MESSAGES);

    const meta = {
      savedAt: Date.now(),
      version: 1,
    };

    localStorage.setItem(STORAGE_KEY, JSON.stringify(persistable));
    localStorage.setItem(META_KEY, JSON.stringify(meta));
  } catch (error) {
    console.warn('保存对话到本地缓存失败:', error);
  }
}

export function loadMessages() {
  try {
    const metaStr = localStorage.getItem(META_KEY);
    if (metaStr) {
      const meta = JSON.parse(metaStr);
      const ageMs = Date.now() - meta.savedAt;
      if (ageMs > MAX_AGE_DAYS * 24 * 60 * 60 * 1000) {
        // 缓存过期，清理掉
        clearMessages();
        return null;
      }
    }

    const data = localStorage.getItem(STORAGE_KEY);
    if (!data) return null;

    const messages = JSON.parse(data);
    if (!Array.isArray(messages) || messages.length === 0) return null;

    return messages.map((msg) => ({
      ...msg,
      isStreaming: false,
    }));
  } catch (error) {
    console.warn('从本地缓存读取对话失败:', error);
    return null;
  }
}

export function clearMessages() {
  try {
    localStorage.removeItem(STORAGE_KEY);
    localStorage.removeItem(META_KEY);
  } catch (error) {
    console.warn('清除本地缓存失败:', error);
  }
}

export function getStorageInfo() {
  try {
    const metaStr = localStorage.getItem(META_KEY);
    if (!metaStr) return null;

    const meta = JSON.parse(metaStr);
    const dataStr = localStorage.getItem(STORAGE_KEY) || '[]';
    const size = new Blob([dataStr]).size;

    return {
      savedAt: meta.savedAt,
      messageCount: JSON.parse(dataStr).length,
      sizeKB: (size / 1024).toFixed(2),
    };
  } catch {
    return null;
  }
}
