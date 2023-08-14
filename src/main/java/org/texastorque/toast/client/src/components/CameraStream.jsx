import { useState, useEffect } from "react";
import axios from "axios";
const CameraStream = (props) => {
  const [imageString, setImageString] = useState("");
  const [index, setIndex] = useState(0);

  useEffect(() => {
    const fetchData = async () => {
      const data = await fetch(`http://127.0.0.1:5000/getCameraStream`);
      const data2 = await data.json();
      console.log(data2)
      setImageString(data2);
      setIndex(index + 1);
    };

    fetchData();
  });
  return (
    <img
      src={`data:image/jpeg;base64,${imageString}`}
      alt="Camera Stream"
      width="656"
      height="369"
      className="stream"
    ></img>
  );
};

export default CameraStream;
