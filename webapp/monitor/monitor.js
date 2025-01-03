// Hàm cập nhật trạng thái
const updateLoadingStatus = (text) => {
    document.getElementById("loading-status").innerText = text;
  };
  
  // Kết nối tới ROS
  const ros = new ROSLIB.Ros({
    url: 'ws://192.168.1.100:9090'
  });
  ros.on('connection', () => console.log('Connected to ROSBridge'));
  ros.on('error', (error) => {
    console.error('Error connecting to ROSBridge:', error);
    updateLoadingStatus('Kết nối ROS thất bại!');
  });
  
  // Map
  const canvas = document.getElementById('map-canvas');
  const ctx = canvas.getContext('2d');
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight;
  
  // Biến lưu thông tin pan và zoom
  let panOffset = { x: 0, y: 0 }; // Tọa độ pan
  let isPanning = false; // Trạng thái pan
  let startPan = { x: 0, y: 0 }; // Tọa độ chuột bắt đầu pan
  let zoomScale = 1; // Tỷ lệ zoom
  
  // Biến lưu thông điệp bản đồ và tf_static
  let lastMapMessage = null;
  let tfStaticData = [];
  
  // Hàm vẽ bản đồ
  function drawMap() {
    if (!lastMapMessage) return;
  
    const { data, info } = lastMapMessage;
    const { width: mapWidth, height: mapHeight } = info;
    const cellWidth = (canvas.width / mapWidth) * zoomScale;
    const cellHeight = (canvas.height / mapHeight) * zoomScale;
  
    // Xóa màn hình và dịch chuyển theo pan
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.save();
    ctx.translate(panOffset.x, panOffset.y);
  
    // Vẽ các ô
    for (let y = 0; y < mapHeight; y++) {
      for (let x = 0; x < mapWidth; x++) {
        const index = y * mapWidth + x;
        const value = data[index];
        let color;
        switch (value) {
          case -1:
            color = 'gray';
            break;
          case 0:
            color = 'white';
            break;
          default:
            color = 'black';
        }
        ctx.fillStyle = color;
        ctx.fillRect(
          x * cellWidth,
          y * cellHeight,
          cellWidth,
          cellHeight
        );
      }
    }
  
    // Vẽ các khung tọa độ từ tf_static
    tfStaticData.forEach((tf) => {
      const { translation, child_frame_id } = tf.transform;
      const { x, y } = translation;
  
      // Chuyển đổi tọa độ tf_static sang canvas
      const canvasX = x * cellWidth;
      const canvasY = y * cellHeight;
  
      ctx.fillStyle = 'red';
      ctx.beginPath();
      ctx.arc(canvasX, canvasY, 5, 0, 2 * Math.PI); // Vẽ điểm
      ctx.fill();
      ctx.fillStyle = 'black';
      ctx.font = '12px Arial';
      ctx.fillText(child_frame_id, canvasX + 5, canvasY - 5); // Hiển thị tên khung
    });
  
    ctx.restore();
  }
  
  // Xử lý sự kiện nhận bản đồ từ ROS
  const mapTopic = new ROSLIB.Topic({
    ros,
    name: '/map',
    messageType: 'nav_msgs/OccupancyGrid'
  });
  mapTopic.subscribe((message) => {
    if (message.data?.length > 0) {
      updateLoadingStatus("");
      lastMapMessage = message;
      drawMap();
    } else {
      updateLoadingStatus('Đang tải bản đồ...');
    }
  });
  
  // Xử lý dữ liệu tf_static từ ROS
  const tfStaticTopic = new ROSLIB.Topic({
    ros,
    name: '/tf_static',
    messageType: 'tf2_msgs/TFMessage'
  });
  tfStaticTopic.subscribe((message) => {
    tfStaticData = message.transforms; // Lưu toàn bộ các khung tf_static
    drawMap(); // Vẽ lại bản đồ để hiển thị tf_static
  });
  
  // Sự kiện pan
  canvas.addEventListener('mousedown', (event) => {
    isPanning = true;
    startPan = { x: event.clientX - panOffset.x, y: event.clientY - panOffset.y };
  });
  
  canvas.addEventListener('mousemove', (event) => {
    if (isPanning) {
      panOffset.x = event.clientX - startPan.x;
      panOffset.y = event.clientY - startPan.y;
      drawMap();
    }
  });
  
  canvas.addEventListener('mouseup', () => {
    isPanning = false;
  });
  
  canvas.addEventListener('mouseleave', () => {
    isPanning = false;
  });
  
  // Sự kiện zoom
  canvas.addEventListener('wheel', (event) => {
    event.preventDefault();
    const zoomFactor = event.deltaY < 0 ? 1.1 : 0.9; // Zoom in or out
    const mouseX = event.clientX - canvas.offsetLeft;
    const mouseY = event.clientY - canvas.offsetTop;
  
    // Tính toán lại vị trí pan để giữ vị trí chuột cố định khi zoom
    panOffset.x -= (mouseX - panOffset.x) * (zoomFactor - 1);
    panOffset.y -= (mouseY - panOffset.y) * (zoomFactor - 1);
  
    zoomScale *= zoomFactor;
    drawMap();
  });
  
  // Xử lý thay đổi kích thước màn hình
  window.addEventListener('resize', () => {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
    drawMap();
  });
  