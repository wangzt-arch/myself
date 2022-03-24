import React, { useState } from "react";
import ReactMarkdown from "react-markdown";
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
          <ReactMarkdown children={doc.value}></ReactMarkdown>
        </div>
      </div>
    </div>
  );
}

export default Docs;
