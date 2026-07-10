const STORAGE_KEY = 'ai-chat-sessions';
const META_KEY = 'ai-chat-meta';
const MAX_SESSIONS = 50;
const MAX_AGE_DAYS = 30;

function getDefaultSession() {
  return {
    id: 'session-' + Date.now(),
    title: '新对话',
    messages: [
      {
        id: '1',
        role: 'assistant',
        content: '你好，我是你的 AI 助手。有什么可以帮到你吗？\n\n基于 DeepSeek-R1 模型，支持深度推理和实时流式响应。',
        isStreaming: false,
      },
    ],
    createdAt: Date.now(),
    updatedAt: Date.now(),
  };
}

function extractTitle(messages) {
  const userMsg = messages.find((msg) => msg.role === 'user');
  if (!userMsg) return '新对话';
  const title = userMsg.content.slice(0, 20);
  return title.length < userMsg.content.length ? title + '...' : title;
}

export function saveSessions(sessions, currentSessionId) {
  try {
    const persistable = sessions
      .map((session) => ({
        ...session,
        messages: session.messages
          .filter((msg) => !msg.isStreaming && msg.content)
          .slice(-100),
        title: session.title || extractTitle(session.messages),
      }))
      .sort((a, b) => b.updatedAt - a.updatedAt)
      .slice(0, MAX_SESSIONS);

    const meta = {
      savedAt: Date.now(),
      version: 2,
      currentSessionId,
    };

    localStorage.setItem(STORAGE_KEY, JSON.stringify(persistable));
    localStorage.setItem(META_KEY, JSON.stringify(meta));
  } catch (error) {
    console.warn('保存会话到本地缓存失败:', error);
  }
}

export function loadSessions() {
  try {
    const metaStr = localStorage.getItem(META_KEY);
    if (metaStr) {
      const meta = JSON.parse(metaStr);
      const ageMs = Date.now() - meta.savedAt;
      if (ageMs > MAX_AGE_DAYS * 24 * 60 * 60 * 1000) {
        clearSessions();
        return { sessions: [getDefaultSession()], currentSessionId: null };
      }
    }

    const data = localStorage.getItem(STORAGE_KEY);
    if (!data) {
      const defaultSession = getDefaultSession();
      return { sessions: [defaultSession], currentSessionId: defaultSession.id };
    }

    const sessions = JSON.parse(data);
    if (!Array.isArray(sessions) || sessions.length === 0) {
      const defaultSession = getDefaultSession();
      return { sessions: [defaultSession], currentSessionId: defaultSession.id };
    }

    const restored = sessions.map((session) => ({
      ...session,
      messages: session.messages.map((msg) => ({
        ...msg,
        isStreaming: false,
      })),
    }));

    const meta = metaStr ? JSON.parse(metaStr) : {};
    const currentSessionId =
      meta.currentSessionId && restored.find((s) => s.id === meta.currentSessionId)
        ? meta.currentSessionId
        : restored[0].id;

    return { sessions: restored, currentSessionId };
  } catch (error) {
    console.warn('从本地缓存读取会话失败:', error);
    const defaultSession = getDefaultSession();
    return { sessions: [defaultSession], currentSessionId: defaultSession.id };
  }
}

export function clearSessions() {
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
    const sessions = JSON.parse(dataStr);

    return {
      savedAt: meta.savedAt,
      sessionCount: sessions.length,
      sizeKB: (size / 1024).toFixed(2),
    };
  } catch {
    return null;
  }
}
