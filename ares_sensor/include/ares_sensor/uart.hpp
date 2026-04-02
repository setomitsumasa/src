//
// Created by karisora on 2025/09/12.
//

#ifndef ARES_SENSOR_UART_HPP
#define ARES_SENSOR_UART_HPP

#include <string>
#include <utility>
#include <vector>

class UartReceiver {
    public:
        UartReceiver(const std::string& port, int baud_rate);
        ~UartReceiver();

        // (id, data)形式でデータを受信（バイナリ形式）
        std::pair<uint8_t, uint32_t> receive();

        // テキスト形式（CSV）でデータを読み取る: "ID,DATA\n"
        // タイムアウトの場合は空文字列を返す
        std::string readLine();

        // CSV形式の文字列をパースしてIDとデータに分ける
        // 正常にパースできたら true を返し、id と data に値を格納する
        // フォーマットエラーなどでパースできなければ false を返す
        bool parseCsvLine(const std::string& line, int & id, double & data);

        bool isOpen() const;

    private:
        int fd_;
};

#endif //ARES_SENSOR_UART_HPP