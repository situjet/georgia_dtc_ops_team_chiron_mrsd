from cv_bridge import CvBridge
import cv2
import subprocess
import numpy as np
import time
import threading
import statistics

#class to handle all the rtsp retrieval functions and stuff
class Streaming:
    rtsp_url = 'rtsp://10.3.1.124:8554/ghadron'
    WIDTH = 640
    HEIGHT = 512
    retry_max = 10
    retry_delay = 2.0
    fps = 6
    is_running = True
    retry_count = 0
    process = None
    frame_count = 0
    last_log_time = time.time()
    last_frame_time = 0
    bridge = CvBridge()
    
    # 性能指标收集
    timing_stats = {
        'total': [],
        'process_check': [],
        'read': [],
        'chunk_read': [],
        'chunk_copy': [],
        'np_convert': [],
    }
    timing_window = 100  # 收集统计数据的帧数
    
    def get_ffmpeg_cmd(self):
        return [
            "ffmpeg",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-rtsp_transport", "tcp",
            "-stimeout", "5000000",    
            "-use_wallclock_as_timestamps", "1",
            "-avioflags", "direct",
            "-flush_packets", "1",
            "-probesize", "32",        # Reduce initial buffering
            "-analyzeduration", "0",   # Skip detailed stream analysis
            "-thread_queue_size", "512", # Increase thread queue size
            "-hwaccel", "auto",        # Use hardware acceleration if available
            "-i", self.rtsp_url,
            "-vsync", "0",
            "-copyts",
            "-vf", f"fps={self.fps},scale={self.WIDTH}:{self.HEIGHT}",
            "-pix_fmt", "bgr24",
            "-f", "rawvideo",
            "-"
        ]

    def log_timing_stats(self):
        """打印性能统计信息"""
        if not all(len(times) > 0 for times in self.timing_stats.values()):
            return
            
        print("\n--- STREAMING PERFORMANCE STATS (last {} frames) ---".format(min(len(self.timing_stats['total']), self.timing_window)))
        for key, times in self.timing_stats.items():
            if len(times) > 0:
                avg = sum(times) / len(times)
                max_t = max(times)
                min_t = min(times)
                if len(times) > 1:
                    std_dev = statistics.stdev(times)
                    print(f"{key.ljust(15)}: avg={avg*1000:.2f}ms, min={min_t*1000:.2f}ms, max={max_t*1000:.2f}ms, std={std_dev*1000:.2f}ms")
                else:
                    print(f"{key.ljust(15)}: avg={avg*1000:.2f}ms, min={min_t*1000:.2f}ms, max={max_t*1000:.2f}ms")
        print("------------------------------------------------------\n")
    
    def add_timing(self, key, value):
        """添加时间测量到统计数据中"""
        self.timing_stats[key].append(value)
        if len(self.timing_stats[key]) > self.timing_window:
            self.timing_stats[key].pop(0)

    def start_ffmpeg(self):
        try:
            cmd = self.get_ffmpeg_cmd()
            
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0
            )
            
            if self.process.poll() is None:
                return True
            else:
                return False
                
        except Exception as e:
            return False

    def retrieve_image(self):
        start_time = time.time()  # Total function time start
        
        frame_size = self.WIDTH * self.HEIGHT * 3
        
        # Pre-allocate the buffer for better performance
        frame_buffer = bytearray(frame_size)
        view = memoryview(frame_buffer)
        
        #checking if the ffmpeg is running
        process_check_time = time.time()
        if not self.process or self.process.poll() is not None:
            process_check_duration = time.time() - process_check_time
            self.add_timing('process_check', process_check_duration)
            
            retry_start = time.time()
            retry_wait = self.retry_delay * (1.5 ** min(self.retry_count, 10))
            self.retry_count += 1
            
            if self.retry_count > 1:
                sleep_start = time.time()
                time.sleep(retry_wait)
                sleep_duration = time.time() - sleep_start
                print(f"Streaming: Retry sleep took {sleep_duration:.6f}s")
            
            start_ffmpeg_time = time.time()
            success = self.start_ffmpeg()
            start_ffmpeg_duration = time.time() - start_ffmpeg_time
            
            if not success:
                retry_duration = time.time() - retry_start
                total_duration = time.time() - start_time
                print(f"Streaming: Failed to start ffmpeg. Retry took {retry_duration:.6f}s, total {total_duration:.6f}s")
                return None, 0
            else:
                self.retry_count = 0
                retry_duration = time.time() - retry_start
                print(f"Streaming: Restarted ffmpeg successfully in {retry_duration:.6f}s (ffmpeg start: {start_ffmpeg_duration:.6f}s)")
        else:
            process_check_duration = time.time() - process_check_time
            self.add_timing('process_check', process_check_duration)
        
        try:
            # More efficient reading using pre-allocated buffer
            read_start_time = time.time()
            bytes_read = 0
            chunk_read_times = []
            chunk_copy_times = []
            
            while bytes_read < frame_size and self.is_running:
                chunk_start = time.time()
                chunk = self.process.stdout.read(frame_size - bytes_read)
                chunk_duration = time.time() - chunk_start
                chunk_read_times.append(chunk_duration)
                
                if not chunk:
                    break
                
                copy_start = time.time()
                view[bytes_read:bytes_read+len(chunk)] = chunk
                copy_duration = time.time() - copy_start
                chunk_copy_times.append(copy_duration)
                
                bytes_read += len(chunk)
            
            read_duration = time.time() - read_start_time
            self.add_timing('read', read_duration)
            
            # 记录平均数据块读取和复制时间
            if chunk_read_times:
                avg_chunk_read = sum(chunk_read_times) / len(chunk_read_times)
                self.add_timing('chunk_read', avg_chunk_read)
            if chunk_copy_times:
                avg_chunk_copy = sum(chunk_copy_times) / len(chunk_copy_times)
                self.add_timing('chunk_copy', avg_chunk_copy)
            
            if bytes_read != frame_size:
                print(f"Streaming: Misalignment in bytes! Read {bytes_read}/{frame_size}, took {read_duration:.6f}s")
                return None, 0

            # Get timestamp when frame is received
            timestamp = time.time()
            
            #return the image frame and timestamp
            np_convert_start = time.time()
            frame = np.frombuffer(frame_buffer, np.uint8).reshape((self.HEIGHT, self.WIDTH, 3))
            np_convert_duration = time.time() - np_convert_start
            self.add_timing('np_convert', np_convert_duration)
            
            total_duration = time.time() - start_time
            self.add_timing('total', total_duration)
            
            self.frame_count += 1
            
            # 每10帧打印一次统计信息
            if self.frame_count % 10 == 0:
                self.log_timing_stats()
                print(f"Streaming: Frame {self.frame_count} retrieved in {total_duration*1000:.2f}ms (read: {read_duration*1000:.2f}ms, np convert: {np_convert_duration*1000:.2f}ms)")
            
            return frame, timestamp
            
        except Exception as e:
            total_duration = time.time() - start_time
            print(f"Streaming: Exception during retrieve: {str(e)}, took {total_duration:.6f}s")
            return None, 0

    def shutdown(self):
        self.is_running = False
        
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
            
        # 打印最终统计信息
        self.log_timing_stats()

        
        
