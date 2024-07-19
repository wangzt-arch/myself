import './index.css'


const text = ['下载xml', '下载图片']


export default function positionsTools(props) {
  const { lf } = props

  const onclickByIndex = (index) => {
    //下载xml
    if (index === 0) {
      downloadXml()
    }
    //下载图片
    if (index === 1) {
      lf.getSnapshot();
    }
  }
  const downloadXml = () => {
    const data = lf.getGraphData()

    console.log('data,,,,,',data);
    download('logic-flow.xml', data);
  }
  const download = (filename,text) => {
    let element = document.createElement('a');
    const jsonStr = JSON.stringify(text);
    const blob = new Blob([jsonStr], { type: "application/json" });
    const objectUrl = URL.createObjectURL(blob);
    element.href = objectUrl;
    element.setAttribute('download', filename);

    element.style.display = 'none';
    document.body.appendChild(element);
    element.click();
    document.body.removeChild(element);
  }

  const uploadXml = (ev) => {
    const file = ev.target.files[0];
    const reader = new FileReader();
    reader.onload = (event) => {
      if (event.target) {
        const xml = event.target.result
        console.log('xml',xml);
        lf.render(JSON.parse(xml));
      }
    };
    reader.readAsText(file); // you could also read images and other binaries
  }

  return (
    <div className="save-tools">
      {
        text.map((item, index) => {
          return <div className='download' onClick={() => onclickByIndex(index)} key={index} title={item}>
            {index === 0 ? <div className="download-xml"></div> : <div className="download-img"></div>}
          </div>
        })
      }
      <div title="上传 XML" className='upload-xml'>
        <input
          type="file"
          className="upload"
          onChange={(ev) => uploadXml(ev)}
        />
      </div>

    </div>
  )
}
