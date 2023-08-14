import { Button } from "react-bootstrap";
import Images from "../imgs/img-directory";
const LSPButton = (props) => {
  const aprilTagID = props.cameraName.includes("local")
    ? props.cameraName.split("local")[1]
    : props.cameraName.split("remote")[1];
  return (
    <div>
      <Button
        className={`${
          props.selectedCamera === props.cameraName
            ? "lsp-button-text-selected"
            : "lsp-button-text"
        }`}
        onClick={(e) => {
          e.preventDefault();
          props.setSelectedCamera(props.cameraName);
        }}
      >
        <img
          src={
            props.cameraType === "april-tags"
              ? Images["tag" + aprilTagID]
              : Images["coneOutlined"]
          }
          alt="camid"
          className="lsp-button-img"
        />
        {props.cameraName}
      </Button>
    </div>
  );
};

export default LSPButton;
