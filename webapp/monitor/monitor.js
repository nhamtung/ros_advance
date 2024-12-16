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
  const mapTopic = new ROSLIB.Topic({
    ros,
    name: '/map',
    messageType: 'nav_msgs/OccupancyGrid'
  });
  window.addEventListener('resize', () => {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
  });
  mapTopic.subscribe((message) => {
    if (message.data?.length > 0) {
      updateLoadingStatus("");
      console.log('OccupancyGrid received:', message);
      const { data, info } = message;
      const { width: mapWidth, height: mapHeight } = info;
      const cellWidth = canvas.width / mapWidth;
      const cellHeight = canvas.height / mapHeight;
      ctx.clearRect(0, 0, canvas.width, canvas.height);
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
          ctx.fillRect(x * cellWidth, y * cellHeight, cellWidth, cellHeight);
        }
      }
    } else {
      updateLoadingStatus('Đang tải bản đồ...');
    }
  });