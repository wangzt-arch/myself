import React, { useEffect, useMemo, useState } from "react";
import ReactMarkdown from "react-markdown";
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { base16AteliersulphurpoolLight } from "react-syntax-highlighter/dist/esm/styles/prism";
import gfm from "remark-gfm";
import Header from "../../components/Header";
import docs from "../../docs";
import "./index.scss";

const slugify = (text) =>
  String(text)
    .trim()
    .toLowerCase()
    .replace(/\s+/g, "-")
    .replace(/[^\w\u4e00-\u9fa5-]/g, "");

const getPlainText = (children) =>
  React.Children.toArray(children)
    .map((child) => {
      if (typeof child === "string" || typeof child === "number") return child;
      if (child?.props?.children) return getPlainText(child.props.children);
      return "";
    })
    .join("");

const getHeadings = (markdown) => {
  return markdown
    .split("\n")
    .map((line) => {
      const match = /^(#{1,3})\s+(.+)$/.exec(line.trim());
      if (!match) return null;
      const text = match[2].replace(/[#`*_]+/g, "").trim();
      return {
        id: slugify(text),
        text,
        level: match[1].length,
      };
    })
    .filter(Boolean);
};

const getReadingMinutes = (markdown) => {
  const text = markdown.replace(/```[\s\S]*?```/g, "").replace(/[#>*`-]/g, "");
  return Math.max(1, Math.ceil(text.length / 500));
};

function Docs() {
  const [currentIndex, setCurrentIndex] = useState(0);
  const [keyword, setKeyword] = useState("");
  const [copiedCode, setCopiedCode] = useState("");

  const categories = useMemo(() => {
    return docs.reduce((result, item, index) => {
      const category = item.category || "未分类";
      if (!result[category]) result[category] = [];
      result[category].push({ ...item, index });
      return result;
    }, {});
  }, []);

  const filteredCategories = useMemo(() => {
    const value = keyword.trim().toLowerCase();
    if (!value) return categories;

    return Object.entries(categories).reduce((result, [category, items]) => {
      const filtered = items.filter((item) => {
        const searchable = `${item.title} ${item.category} ${(item.tags || []).join(" ")} ${item.value}`.toLowerCase();
        return searchable.includes(value);
      });
      if (filtered.length) result[category] = filtered;
      return result;
    }, {});
  }, [categories, keyword]);

  useEffect(() => {
    const visibleItems = Object.values(filteredCategories).flat();
    const isCurrentVisible = visibleItems.some((item) => item.index === currentIndex);

    if (!isCurrentVisible && visibleItems.length) {
      setCurrentIndex(visibleItems[0].index);
    }
  }, [currentIndex, filteredCategories]);

  const doc = docs[currentIndex] || docs[0];
  const headings = useMemo(() => getHeadings(doc.value), [doc]);
  const readingMinutes = useMemo(() => getReadingMinutes(doc.value), [doc]);

  const copyCode = async (code) => {
    try {
      await navigator.clipboard.writeText(code);
      setCopiedCode(code);
      window.setTimeout(() => setCopiedCode(""), 1200);
    } catch (error) {
      console.warn("Copy failed", error);
    }
  };

  const scrollToHeading = (id) => {
    const container = document.querySelector(".docs-content");
    const target = document.getElementById(id);

    if (!container || !target) return;

    const containerTop = container.getBoundingClientRect().top;
    const targetTop = target.getBoundingClientRect().top;

    container.scrollTo({
      top: container.scrollTop + targetTop - containerTop - 18,
      behavior: "smooth",
    });
  };

  return (
    <div className="docs-page">
      <Header />
      <div className="docs">
        <aside className="docs-sidebar">
          <div className="docs-search">
            <input
              value={keyword}
              onChange={(event) => setKeyword(event.target.value)}
              placeholder="搜索文档、标签、正文"
            />
          </div>

          <div className="docs-nav">
            {Object.keys(filteredCategories).length ? (
              Object.entries(filteredCategories).map(([category, items]) => (
                <section className="docs-category" key={category}>
                  <h3>{category}</h3>
                  {items.map((item) => (
                    <button
                      className={currentIndex === item.index ? "docs-title_item docs-title_item--active" : "docs-title_item"}
                      key={item.title}
                      type="button"
                      onClick={() => setCurrentIndex(item.index)}
                    >
                      <span>{item.title}</span>
                      <small>{(item.tags || []).join(" / ")}</small>
                    </button>
                  ))}
                </section>
              ))
            ) : (
              <p className="docs-empty">没有匹配的文档</p>
            )}
          </div>
        </aside>

        <main className="docs-content">
          <div className="docs-article-meta">
            <span>{doc.category}</span>
            <span>{readingMinutes} 分钟阅读</span>
          </div>
          <h1 className="docs-article-title">{doc.title}</h1>
          <div className="docs-tags">
            {(doc.tags || []).map((tag) => (
              <span key={tag}>{tag}</span>
            ))}
          </div>

          <article className="markdown-body">
            <ReactMarkdown
              children={doc.value}
              remarkPlugins={[gfm]}
              components={{
                h1({ children }) {
                  const text = getPlainText(children);
                  return <h1 id={slugify(text)}>{children}</h1>;
                },
                h2({ children }) {
                  const text = getPlainText(children);
                  return <h2 id={slugify(text)}>{children}</h2>;
                },
                h3({ children }) {
                  const text = getPlainText(children);
                  return <h3 id={slugify(text)}>{children}</h3>;
                },
                code({ inline, className, children, ...props }) {
                  const match = /language-(\w+)/.exec(className || "");
                  const code = String(children).replace(/\n$/, "");
                  return !inline && match ? (
                    <div className="code-block">
                      <button type="button" onClick={() => copyCode(code)}>
                        {copiedCode === code ? "已复制" : "复制"}
                      </button>
                      <SyntaxHighlighter
                        children={code}
                        style={base16AteliersulphurpoolLight}
                        language={match[1]}
                        PreTag="div"
                        {...props}
                      />
                    </div>
                  ) : (
                    <code className={className} {...props}>
                      {children}
                    </code>
                  );
                },
              }}
            />
          </article>
        </main>

        <aside className="docs-toc">
          <h3>目录</h3>
          {headings.length ? (
            headings.map((item) => (
              <button
                className={`toc-link toc-link--${item.level}`}
                key={`${item.id}-${item.text}`}
                type="button"
                onClick={() => scrollToHeading(item.id)}
              >
                {item.text}
              </button>
            ))
          ) : (
            <span className="toc-empty">暂无目录</span>
          )}
        </aside>
      </div>
    </div>
  );
}

export default Docs;
