#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <variant>
#include <stdexcept>

// Minimal JSON Parser for Configuration
// Supports: Object, String, Number, Boolean, Null
// Does NOT support: Array (simplification for config map), Escaped characters in strings (basic)

namespace MiniJSON {

enum class Type { Null, Object, String, Number, Boolean };

struct Value;
using Object = std::map<std::string, Value>;

struct Value {
    Type type = Type::Null;
    std::string s_val;
    double n_val = 0.0;
    bool b_val = false;
    Object o_val;

    Value() = default;
    Value(std::string s) : type(Type::String), s_val(s) {}
    Value(double n) : type(Type::Number), n_val(n) {}
    Value(bool b) : type(Type::Boolean), b_val(b) {}
    Value(Object o) : type(Type::Object), o_val(o) {}

    // Helpers
    std::string as_string() const {
        if (type == Type::String) return s_val;
        if (type == Type::Number) {
            std::string s = std::to_string(n_val);
            // Remove trailing zeros for cleanliness if it's an integer
            s.erase(s.find_last_not_of('0') + 1, std::string::npos);
            if(s.back() == '.') s.pop_back();
            return s;
        }
        if (type == Type::Boolean) return b_val ? "true" : "false";
        return "";
    }
    
    double as_double() const {
        if (type == Type::Number) return n_val;
        return 0.0;
    }

    bool as_bool() const {
        if (type == Type::Boolean) return b_val;
        return false;
    }

    bool is_null() const { return type == Type::Null; }
};

class Parser {
    const std::string& str;
    size_t pos = 0;

    void skip_whitespace() {
        while (pos < str.size() && std::isspace(str[pos])) pos++;
    }

    char peek() {
        skip_whitespace();
        if (pos >= str.size()) return 0;
        return str[pos];
    }

    char get() {
        char c = peek();
        if (c) pos++;
        return c;
    }

    void expect(char c) {
        if (get() != c) throw std::runtime_error(std::string("Expected '") + c + "'");
    }

    std::string parse_string() {
        expect('"');
        std::string s;
        while (pos < str.size()) {
            char c = str[pos++];
            if (c == '"') return s;
            if (c == '\\') {
                // formatting simplified, just skip escape
                if (pos < str.size()) s += str[pos++];
            } else {
                s += c;
            }
        }
        throw std::runtime_error("Unterminated string");
    }

    Value parse_number() {
        size_t start = pos;
        if (peek() == '-') pos++;
        while (pos < str.size() && (isdigit(str[pos]) || str[pos] == '.' || str[pos] == 'e' || str[pos] == 'E' || str[pos] == '+' || str[pos] == '-')) pos++;
        return std::stod(str.substr(start, pos - start));
    }

    Value parse_value() {
        char c = peek();
        if (c == '{') return parse_object();
        if (c == '"') return parse_string();
        if (isdigit(c) || c == '-') return parse_number();
        if (str.substr(pos, 4) == "true") { pos += 4; return true; }
        if (str.substr(pos, 5) == "false") { pos += 5; return false; }
        if (str.substr(pos, 4) == "null") { pos += 4; return Value(); }
        throw std::runtime_error(std::string("Unexpected character: ") + c);
    }

    Value parse_object() {
        expect('{');
        Object obj;
        char c = peek();
        if (c == '}') {
            get();
            return obj;
        }
        while (true) {
            std::string key = parse_string();
            expect(':');
            obj[key] = parse_value();
            c = peek();
            if (c == '}') {
                get();
                break;
            }
            expect(',');
        }
        return obj;
    }

public:
    Parser(const std::string& s) : str(s) {}
    Value parse() { return parse_value(); }
};

inline Value Parse(const std::string& s) {
    Parser p(s);
    return p.parse();
}

} // namespace MiniJSON

class DemoConfiguration {
public:
    MiniJSON::Value root;

    bool Load(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) return false;
        std::stringstream buffer;
        buffer << f.rdbuf();
        try {
            root = MiniJSON::Parse(buffer.str());
            return true;
        } catch (std::exception& e) {
            std::cerr << "JSON Parse Error: " << e.what() << std::endl;
            return false;
        }
    }

    // Get helper with dot notation "vehicle.parameters.step_size"
    MiniJSON::Value Get(const std::string& path) const {
        std::string seg;
        std::stringstream ss(path);
        const MiniJSON::Value* curr = &root;

        while (std::getline(ss, seg, '.')) {
            if (curr->type != MiniJSON::Type::Object) return MiniJSON::Value(); // Not found or not object
            auto it = curr->o_val.find(seg);
            if (it == curr->o_val.end()) return MiniJSON::Value(); // Not found
            curr = &it->second;
        }
        return *curr;
    }

    std::string GetString(const std::string& path, const std::string& def) const {
        auto v = Get(path);
        if (v.type == MiniJSON::Type::String) return v.s_val;
        return def;
    }

    double GetDouble(const std::string& path, double def) const {
        auto v = Get(path);
        if (v.type == MiniJSON::Type::Number) return v.n_val;
        return def;
    }

    bool GetBool(const std::string& path, bool def) const {
        auto v = Get(path);
        if (v.type == MiniJSON::Type::Boolean) return v.b_val;
        return def;
    }
};
