#pragma once

#include "cv/translation-calibrator.hpp"
#include "api/plugin-support.hpp"
#include "compat/util.hpp"
#include "compat/euler.hpp"

#include <tuple>
#include <iterator>

#include <QObject>

namespace calibrator_wizard {

template<int j, int k> using mat = Mat<double, j, k>;
template<int k> using vec = Mat<double, k, 1>;

using pose_t = vec<6>;
using euler_angles_t = vec<3>;
using rmat = euler::rmat;
using euler::euler_to_rmat;

enum yaw_action {
    y_nada, y_center, y_left, y_right
};

enum pitch_action {
    p_nada, p_center, p_up, p_down
};

using tuple = std::tuple<yaw_action, pitch_action>;

class Wizard final : QObject
{
    Q_OBJECT

    TranslationCalibrator calibrator;

    static constexpr tuple actions[] =
    {
        // centered, yaw left/right
        { y_left, p_center, },
        { y_center, p_center },
        { y_right, p_center },

        // upward, yaw left/right
        { y_center, p_center },
        { y_center, p_up },
        { y_left, p_up },
        { y_center, p_up },
        { y_right, p_up },

        // downward, yaw left/right
        { y_center, p_center },
        { y_center, p_down },
        { y_left, p_down },
        { y_center, p_down },
        { y_right, p_down },

        // done
        { y_center, p_center },
    };

    using iterator_t = decltype(std::begin(actions));
    iterator_t iter;

    using bb = std::tuple<bool, bool>;
    bb reached_p;

    tuple expected() const;

public:
    tuple cur_pose(const pose_t& pose) const;
    tuple pose_diff(const tuple& pose) const;
    bool maybe_next(const pose_t& pose);
    bool is_done() const;

    static QString action_to_string(const tuple& pose);

    Wizard(QObject* = nullptr);
};

} // ns calibrator_wizard
