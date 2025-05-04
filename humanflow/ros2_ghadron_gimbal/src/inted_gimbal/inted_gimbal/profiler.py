import time
import functools
import threading
import psutil
import os
import cProfile
import pstats
import io
from collections import defaultdict
import logging
import traceback

class FunctionProfiler:
    """Profile individual function execution times"""
    def __init__(self):
        self.stats = defaultdict(lambda: {'calls': 0, 'total_time': 0, 'min_time': float('inf'), 'max_time': 0})
        self.lock = threading.Lock()
        
    def __call__(self, func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            start_time = time.time()
            try:
                result = func(*args, **kwargs)
                return result
            finally:
                elapsed = time.time() - start_time
                with self.lock:
                    stats = self.stats[func.__name__]
                    stats['calls'] += 1
                    stats['total_time'] += elapsed
                    stats['min_time'] = min(stats['min_time'], elapsed)
                    stats['max_time'] = max(stats['max_time'], elapsed)
        return wrapper
    
    def get_stats(self):
        with self.lock:
            return {k: {
                'calls': v['calls'], 
                'total_time': v['total_time'], 
                'avg_time': v['total_time'] / v['calls'] if v['calls'] > 0 else 0,
                'min_time': v['min_time'] if v['min_time'] != float('inf') else 0,
                'max_time': v['max_time']
            } for k, v in self.stats.items()}
            
class ResourceMonitor:
    """Monitor system resources consumption"""
    def __init__(self, interval=1.0):
        self.interval = interval
        self.running = False
        self.stats = {
            'cpu': [],
            'memory': [],
            'threads': [],
            'timestamps': []
        }
        self.process = psutil.Process(os.getpid())
        self.thread = None
        self.lock = threading.Lock()
        
    def start(self):
        if self.running:
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._monitor_loop)
        self.thread.daemon = True
        self.thread.start()
        
    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
            
    def _monitor_loop(self):
        while self.running:
            try:
                cpu_percent = self.process.cpu_percent(interval=0.1)
                mem_info = self.process.memory_info()
                thread_count = threading.active_count()
                
                with self.lock:
                    self.stats['cpu'].append(cpu_percent)
                    self.stats['memory'].append(mem_info.rss / 1024 / 1024)  # MB
                    self.stats['threads'].append(thread_count)
                    self.stats['timestamps'].append(time.time())
            except Exception as e:
                logging.error(f"Error in resource monitoring: {e}")
                traceback.print_exc()
                
            time.sleep(self.interval)
            
    def get_stats(self):
        with self.lock:
            return {
                'cpu': {
                    'avg': sum(self.stats['cpu']) / len(self.stats['cpu']) if self.stats['cpu'] else 0,
                    'max': max(self.stats['cpu']) if self.stats['cpu'] else 0,
                    'latest': self.stats['cpu'][-1] if self.stats['cpu'] else 0,
                    'history': self.stats['cpu']
                },
                'memory': {
                    'avg': sum(self.stats['memory']) / len(self.stats['memory']) if self.stats['memory'] else 0,
                    'max': max(self.stats['memory']) if self.stats['memory'] else 0,
                    'latest': self.stats['memory'][-1] if self.stats['memory'] else 0,
                    'history': self.stats['memory']
                },
                'threads': {
                    'avg': sum(self.stats['threads']) / len(self.stats['threads']) if self.stats['threads'] else 0,
                    'max': max(self.stats['threads']) if self.stats['threads'] else 0,
                    'latest': self.stats['threads'][-1] if self.stats['threads'] else 0,
                    'history': self.stats['threads']
                },
                'timestamps': self.stats['timestamps']
            }
            
class ProfileManager:
    """Manage all profiling activities"""
    def __init__(self, output_dir=None):
        self.function_profiler = FunctionProfiler()
        self.resource_monitor = ResourceMonitor()
        self.profilers = {}
        self.output_dir = output_dir or os.path.expanduser('~/profiling_results')
        os.makedirs(self.output_dir, exist_ok=True)
        
    def start_profiling(self):
        self.resource_monitor.start()
        
        # Start cProfile for global profiling
        self.profilers['global'] = cProfile.Profile()
        self.profilers['global'].enable()
        
    def stop_profiling(self):
        # Stop cProfile
        if 'global' in self.profilers:
            self.profilers['global'].disable()
            
        # Stop resource monitoring
        self.resource_monitor.stop()
        
    def save_results(self):
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        
        # Save cProfile results
        if 'global' in self.profilers:
            s = io.StringIO()
            ps = pstats.Stats(self.profilers['global'], stream=s).sort_stats('cumulative')
            ps.print_stats(30)  # Print top 30 functions
            
            with open(f"{self.output_dir}/cprofile_{timestamp}.txt", 'w') as f:
                f.write(s.getvalue())
                
        # Save function profiler results
        func_stats = self.function_profiler.get_stats()
        with open(f"{self.output_dir}/function_stats_{timestamp}.txt", 'w') as f:
            f.write("Function Profiling Results:\n")
            f.write("-" * 80 + "\n")
            for func_name, stats in sorted(func_stats.items(), key=lambda x: x[1]['total_time'], reverse=True):
                f.write(f"{func_name}:\n")
                f.write(f"  Calls: {stats['calls']}\n")
                f.write(f"  Total Time: {stats['total_time']:.6f}s\n")
                f.write(f"  Avg Time: {stats['avg_time']:.6f}s\n")
                f.write(f"  Min Time: {stats['min_time']:.6f}s\n")
                f.write(f"  Max Time: {stats['max_time']:.6f}s\n\n")
                
        # Save resource monitoring results
        res_stats = self.resource_monitor.get_stats()
        with open(f"{self.output_dir}/resource_stats_{timestamp}.txt", 'w') as f:
            f.write("Resource Monitoring Results:\n")
            f.write("-" * 80 + "\n")
            f.write(f"CPU Usage:\n")
            f.write(f"  Average: {res_stats['cpu']['avg']:.2f}%\n")
            f.write(f"  Maximum: {res_stats['cpu']['max']:.2f}%\n")
            f.write(f"  Latest: {res_stats['cpu']['latest']:.2f}%\n\n")
            
            f.write(f"Memory Usage:\n")
            f.write(f"  Average: {res_stats['memory']['avg']:.2f} MB\n")
            f.write(f"  Maximum: {res_stats['memory']['max']:.2f} MB\n")
            f.write(f"  Latest: {res_stats['memory']['latest']:.2f} MB\n\n")
            
            f.write(f"Thread Count:\n")
            f.write(f"  Average: {res_stats['threads']['avg']:.2f}\n")
            f.write(f"  Maximum: {res_stats['threads']['max']}\n")
            f.write(f"  Latest: {res_stats['threads']['latest']}\n")
            
        return f"{self.output_dir}/profiling_results_{timestamp}" 