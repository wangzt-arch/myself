import React, { useState } from "react";
import ReactMarkdown from "react-markdown";
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter'
import { base16AteliersulphurpoolLight } from 'react-syntax-highlighter/dist/esm/styles/prism'
import gfm from 'remark-gfm'
import Header from "../../components/Header";
import docs from "../../docs";
import "./index.scss";

function Docs() {
  let [currentIndex, setCurrentIndex] = useState(0);
  let [doc, setDoc] = useState(docs[0]);
  const changeContent = (index) => {
    setCurrentIndex(index)
    setDoc(docs[index]);
  };
  return (
    <div>
      <Header></Header>
      <div className="docs">
        <div className="docs-title">
          {docs.map((item, index) => (
            <div
              onClick={() => changeContent(index)}
              className={currentIndex === index ? "docs-title_item docs-title_item--active" : "docs-title_item"}
              key={index}
            >
              {item.title}
            </div>
          ))}
        </div>
        <div className="docs-content">
          <ReactMarkdown children={doc.value} remarkPlugins={[gfm]}
            components={{
              code({ node, inline, className, children, ...props }) {
                const match = /language-(\w+)/.exec(className || '')
                return !inline && match ? (
                  <SyntaxHighlighter
                    children={String(children).replace(/\n$/, '')}
                    style={base16AteliersulphurpoolLight}
                    language={match[1]}
                    PreTag="div"
                    {...props}
                  />
                ) : (
                  <code className={className} {...props}>
                    {children}
                  </code>
                )
              }
            }}
          ></ReactMarkdown>
        </div>
      </div>
    </div>
  );
}

export default Docs;
