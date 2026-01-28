#include <simpleble/SimpleBLE.h>
#include "hub/Framer.h"
#include "hub/Parser.h"
#include "hub/Pipeline.h"
#include "hub/model/Linear.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

static inline uint64_t now_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

static inline bool starts_with(const std::string& s, const std::string& prefix) {
    return s.size() >= prefix.size() && std::memcmp(s.data(), prefix.data(), prefix.size()) == 0;
}

struct Args {
    std::string prefix = "Softionics";
    int scan_ms = 3000;

    bool ma_on = false;
    size_t ma_win = 5;

    bool ema_on = true;
    float ema_alpha = 0.2f;

    bool notch_on = false;
    double fs = 200.0;
    double notch_f0 = 60.0;
    double notch_q = 30.0;

    bool bias_on = false;
    size_t bias_capture_frames = 200;

    bool model_on = true;
    float model_bias = 0.0f;
    std::string weights_csv;

    std::string csv_path;
};

static Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];

        auto need = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                std::exit(2);
            }
            return argv[++i];
        };

        if (k == "--prefix") a.prefix = need("--prefix");
        else if (k == "--scan_ms") a.scan_ms = std::atoi(need("--scan_ms"));

        else if (k == "--ma") { a.ma_on = true; a.ma_win = static_cast<size_t>(std::strtoul(need("--ma"), nullptr, 10)); }
        else if (k == "--no_ma") a.ma_on = false;

        else if (k == "--ema_alpha") { a.ema_on = true; a.ema_alpha = std::strtof(need("--ema_alpha"), nullptr); }
        else if (k == "--no_ema") a.ema_on = false;

        else if (k == "--notch") { a.notch_on = true; a.notch_f0 = std::strtod(need("--notch"), nullptr); }
        else if (k == "--q") a.notch_q = std::strtod(need("--q"), nullptr);
        else if (k == "--fs") a.fs = std::strtod(need("--fs"), nullptr);
        else if (k == "--no_notch") a.notch_on = false;

        else if (k == "--bias") a.bias_on = true;
        else if (k == "--no_bias") a.bias_on = false;
        else if (k == "--bias_frames") a.bias_capture_frames = static_cast<size_t>(std::strtoul(need("--bias_frames"), nullptr, 10));

        else if (k == "--model") a.model_on = true;
        else if (k == "--no_model") a.model_on = false;
        else if (k == "--model_bias") a.model_bias = std::strtof(need("--model_bias"), nullptr);
        else if (k == "--weights") a.weights_csv = need("--weights");

        else if (k == "--csv") a.csv_path = need("--csv");
        else {
            std::cerr << "Unknown arg: " << k << "\n";
            std::exit(2);
        }
    }
    return a;
}

static std::optional<std::pair<SimpleBLE::BluetoothUUID, SimpleBLE::BluetoothUUID>>
pick_first_notify_char(SimpleBLE::Peripheral& p) {
    auto services = p.services();
    for (auto& s : services) {
        auto chars = s.characteristics();
        for (auto& c : chars) {
            if (c.can_notify()) return std::make_pair(s.uuid(), c.uuid());
        }
    }
    return std::nullopt;
}

int main(int argc, char** argv) {
    Args args = parse_args(argc, argv);

    if (!SimpleBLE::Adapter::bluetooth_enabled()) {
        std::cerr << "Bluetooth not enabled or permission missing\n";
        return 1;
    }

    auto adapters = SimpleBLE::Adapter::get_adapters();
    if (adapters.empty()) {
        std::cerr << "No adapters\n";
        return 1;
    }
    auto adapter = adapters[0];

    std::cout << "Adapter: " << adapter.identifier() << " " << adapter.address() << "\n";

    SimpleBLE::Peripheral chosen;
    bool found = false;

    for (;;) {
        adapter.scan_for(args.scan_ms);
        auto results = adapter.scan_get_results();

        int best_rssi = -32768;
        for (auto& p : results) {
            auto name = p.identifier();
            if (!starts_with(name, args.prefix)) continue;
            int rssi = p.rssi();
            if (!found || rssi > best_rssi) {
                chosen = p;
                best_rssi = rssi;
                found = true;
            }
        }

        if (found) break;
        std::cout << "Scanning... no match yet\n";
    }

    std::cout << "Chosen: " << chosen.identifier() << " " << chosen.address() << " rssi=" << chosen.rssi() << "\n";

    chosen.connect();
    std::cout << "Connected\n";

    auto notify_pair = pick_first_notify_char(chosen);
    if (!notify_pair) {
        std::cerr << "No notify characteristic found\n";
        return 1;
    }

    auto service_uuid = notify_pair->first;
    auto char_uuid = notify_pair->second;

    std::cout << "Notify: service=" << service_uuid << " char=" << char_uuid << "\n";

    hub::LineFramer framer;
    hub::CsvFloatParser parser;

    hub::Pipeline pipe;
    hub::PipelineConfig cfg;
    cfg.enable_ma = args.ma_on;
    cfg.ma_win = args.ma_win;
    cfg.enable_ema = args.ema_on;
    cfg.ema_alpha = args.ema_alpha;
    cfg.enable_notch = args.notch_on;
    cfg.fs_hz = args.fs;
    cfg.notch_f0 = args.notch_f0;
    cfg.notch_q = args.notch_q;
    cfg.enable_bias = args.bias_on;
    cfg.enable_model = args.model_on;
    cfg.model_bias = args.model_bias;
    pipe.set_config(cfg);

    std::optional<std::ofstream> csv;
    if (!args.csv_path.empty()) {
        csv.emplace(args.csv_path, std::ios::binary);
        if (!(*csv)) {
            std::cerr << "CSV open failed: " << args.csv_path << "\n";
            return 1;
        }
    }

    std::atomic<bool> want_bias_capture{false};
    std::atomic<bool> quit{false};

    std::thread input_thread([&]() {
        for (;;) {
            int c = std::getc(stdin);
            if (c == EOF) continue;
            if (c == 'b' || c == 'B') want_bias_capture.store(true);
            if (c == 'q' || c == 'Q') { quit.store(true); break; }
        }
    });

    std::vector<float> weights_loaded;
    bool weights_pending = false;
    if (!args.weights_csv.empty()) {
        auto w = hub::load_weights_csv_1line(args.weights_csv);
        if (w) {
            weights_loaded = *w;
            weights_pending = true;
            std::cout << "Weights loaded: " << weights_loaded.size() << "\n";
        } else {
            std::cerr << "Weights load failed: " << args.weights_csv << "\n";
        }
    }

    uint64_t t0 = now_ns();
    std::atomic<uint64_t> frames_ok{0};
    std::atomic<uint64_t> frames_bad{0};

    chosen.notify(service_uuid, char_uuid, [&](SimpleBLE::ByteArray payload) {
        if (quit.load()) return;

        std::string chunk(payload.begin(), payload.end());
        auto lines = framer.push(chunk);

        for (auto& line : lines) {
            auto v = parser.parse_line(line);
            if (!v) { frames_bad.fetch_add(1); continue; }

            if (pipe.config().enable_model && weights_pending) {
                pipe.ensure_initialized(v->size());
                if (weights_loaded.size() == v->size()) {
                    pipe.set_model_weights(weights_loaded);
                    weights_pending = false;
                }
            }

            if (want_bias_capture.exchange(false)) {
                pipe.ensure_initialized(v->size());
                pipe.begin_bias_capture(args.bias_capture_frames);
                std::cout << "Bias capture started: frames=" << args.bias_capture_frames << "\n";
            }

            uint64_t t = now_ns();
            auto out = pipe.process(t, *v);
            frames_ok.fetch_add(1);

            if (csv) {
                double ts = (static_cast<double>(out.frame.t_ns - t0)) * 1e-9;
                (*csv) << ts;
                for (size_t i = 0; i < out.frame.x.size(); ++i) (*csv) << "," << out.frame.x[i];
                if (out.model_valid) (*csv) << "," << out.model_out;
                (*csv) << "\n";
            }

            static thread_local uint64_t last_print = 0;
            if (t - last_print > 200000000ull) {
                last_print = t;
                std::cout << "N=" << out.frame.x.size()
                          << " ok=" << frames_ok.load()
                          << " bad=" << frames_bad.load();

                if (pipe.bias_capturing()) std::cout << " bias=capturing";
                else if (pipe.bias_has()) std::cout << " bias=on";

                if (out.model_valid) std::cout << " y=" << out.model_out;
                std::cout << "\n";
            }
        }
    });

    std::cout << "Running. keys: b=bias capture, q=quit\n";

    while (!quit.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    chosen.unsubscribe(service_uuid, char_uuid);
    chosen.disconnect();

    if (input_thread.joinable()) input_thread.join();

    std::cout << "Done\n";
    return 0;
}
