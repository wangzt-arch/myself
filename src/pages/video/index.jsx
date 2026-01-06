import React from "react";
import Header from "../../components/Header";
import "./index.scss";

function Docs() {
  const importAll = (r) => {
    let models = [];
    r.keys().forEach((item) => {
      models.push({ name: item.replace('./', ''), value: r(item) });
    });
    return models;
  };
  const videos = importAll(require.context('../../videos', false, /\.mp4$/));

  const videoStyle = {
    objectFit: "fill"
  }


  return (
    <div>
      <Header></Header>
      <div className="video">
        {
          videos.length && videos.map((video, index) => {
            return <div className="video-item" key={index}>
              <video width='100%' height='100%' controls style={videoStyle}>
                <source src={video.value} sizes="100%" type="video/mp4"></source>
              </video>
            </div>
          })
        }

      </div>
    </div >
  );
}

export default Docs;
