import { useState, useEffect } from "react";
import LSPButton from "./components/LSPButton.jsx";
import CameraStream from "./components/CameraStream.jsx";
import CameraIntrinsics from "./components/CameraIntrinsics.jsx";
import Targets from "./components/Targets.jsx";

const Dashboard = () => {
  const [selectedCamera, setSelectedCamera] = useState("local0");
  let [config, setConfig] = useState({});

  useEffect(() => {
    const fetchData = async () => {
      const res = await fetch("http://127.0.0.1:5000/getCameraDispatch");
      config = await res.json();
      console.log(config);
      setConfig(config);
    };
    fetchData();
  }, []);

  return (
    <div className="dashboard">
      <div className="left-side-panel">
        <div className="heading">
          <div className="heading-title">
            <h1 style={{ fontFamily: "Market_Deco", fontSize: "2em" }}>
              Texas T.O.A.S.T.E.R.
            </h1>
          </div>

          <h4 style={{ fontSize: ".75em" }}>
            &copy; 2023 FRC 1477 Texas Torque
          </h4>
        </div>
        {Object.values(config).map((item, index) => {
          //   if (index % 2 !== 0) return;
          return (
            <LSPButton
              key={index}
              cameraName={index <= 4 ? `local${index}` : `remote${index - 5}`}
              cameraType={item}
              selectedCamera={selectedCamera}
              setSelectedCamera={setSelectedCamera}
            />
          );
        })}
      </div>
      <div className="right-side-panel">
        <div className="rsp-grid">
          <div className="rsp-stream">
            <h2>Stream</h2>
            <CameraStream streamID={selectedCamera} />
          </div>
          <div className="rsp-intrins">
            <h2>Intrinsics</h2>
            <CameraIntrinsics />
          </div>
        </div>
        <div className="right-side-panel-info">
          <Targets />
        </div>
      </div>
    </div>
  );
};

export default Dashboard;
