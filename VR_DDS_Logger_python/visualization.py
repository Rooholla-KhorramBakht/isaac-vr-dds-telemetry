import rerun as rr

class RerunVisualizer:
    def __init__(self, app_name="RerunVisualizer", log_time_label='logtime', spawn=True, port=9876):
        # Initialize Rerun session
        if spawn == False:
            rr.init(app_name, spawn=False)
            rr.connect(f'127.0.0.1:{port}')
        else:
            rr.init(app_name, spawn=True)
        self.log_time_label = log_time_label

    def logPoints(self, points, colors=None, radii=None, log_path='/points', log_time=None):
        ps = []
        cs = []
        rs = []
        if colors is None:
            colors = [[0, 255, 0]] * points.shape[0]
        if radii is None:
            radii = [0.002] * points.shape[0]

        rr.log(log_path, rr.Points3D(points, colors = colors, radii=radii))   
        if log_time is not None:
            rr.set_time_seconds(self.log_time_label, log_time)
    
    def logCoordinateFrame(self, world_T_frame, log_path, axis_length=0.2, log_time=None):
        rr.log(
            f"{log_path}",
            rr.Arrows3D(
                vectors=[[axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]],
                colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                radii = [axis_length/30, axis_length/30, axis_length/30]
            ),
        )
        xyzw = R.from_matrix(world_T_frame[0:3,0:3]).as_quat()
        quat = rr.Quaternion.identity()
        quat.xyzw = xyzw
        rr.log(log_path, rr.Transform3D(translation=world_T_frame[:3,-1].squeeze(), rotation=quat))
        if log_time is not None:
            rr.set_time_seconds(self.log_time_label, log_time)