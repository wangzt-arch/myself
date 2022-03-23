import React, { useState } from "react";
import "./index.scss";

function Line() {
  const [text, setText] = useState('swing')
  return (
    <div className="line">
      <input type="checkbox" id="toggle" onChange={() => { text === 'swing' ? setText('learning') : setText('swing') }} />
      <section id="sect">
        <label for="toggle" className="toggle">change view</label>
        <ul className="line-ul">
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
          <li className="line-li">{text}</li>
        </ul>
      </section>
    </div>
  );
}

export default Line;
