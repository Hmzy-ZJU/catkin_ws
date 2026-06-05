#pragma once
#include <string>
#include <fstream>
#include <mutex>

namespace runlog {

class Logger {
public:
  static Logger& Instance() {
    static Logger inst; return inst;
  }
  void open(const std::string& path) {
    std::lock_guard<std::mutex> lk(m_);
    if (ofs_.is_open()) return;
    ofs_.open(path, std::ios::out);
    // CSV 头
    ofs_ << "stamp,track_ms,select_ms,kp_raw,kp_sel,fps_est\n";
    ofs_.flush();
  }
  void write(double stamp, double t_ms, double sel_ms, int kp_raw, int kp_sel, double fps_est) {
    std::lock_guard<std::mutex> lk(m_);
    if (!ofs_.is_open()) return;
    ofs_ << std::fixed;
    ofs_ << stamp << "," << t_ms << "," << sel_ms << ","
         << kp_raw << "," << kp_sel << "," << fps_est << "\n";
  }
private:
  std::ofstream ofs_;
  std::mutex m_;
};

} // namespace runlog
