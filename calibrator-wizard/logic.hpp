#pragma once

#include "cv/translation-calibrator.hpp"
#include "api/plugin-support.hpp"
#include "compat/util.hpp"
#include "compat/euler.hpp"

#include <tuple>
#include <iterator>

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
using calibrator_order = std::tuple<int, int, int>;

template<typename tracker_type, typename = void>
struct tracker_traits
{
    static inline calibrator_order get_calibrator_order()
    {
        return std::make_tuple(0, 1, 2);
    }

    static inline void get_pose(pose_t& euler, cv::Matx33d& R, cv::Vec3d& T)
    {
        euler = pose_t(0, 0, 0, 0, 0, 0);
        R = cv::Matx33d::eye();
        T = cv::Vec3d::zeros();
    }

    static_assert(!std::is_same_v<tracker_type, tracker_type>,
                  "specialize for given tracker type");
};

namespace detail {

class wizard_base
{
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

    cv::Matx33d R;
    cv::Vec3d T;
    pose_t euler;

    TranslationCalibrator make_from_calib_order();

protected:
    virtual void get_pose(pose_t& euler, cv::Matx33d& R, cv::Vec3d& T) const = 0;
    virtual calibrator_order get_calibrator_order() const = 0;
    bool maybe_next();

public:
    tuple cur_pose() const;
    tuple pose_diff() const;
    bool is_done() const;

    static QString action_to_string();

    wizard_base();
    virtual ~wizard_base();
};

} // ns calibrator_wizard::detail

template<typename tracker_type>
class wizard final : public wizard_base
{
    using traits = tracker_traits<tracker_type>;

    std::shared_ptr<const tracker_type> tracker;

    void get_pose(pose_t& euler, cv::Matx33d& R, cv::Vec3d& T) const override;
    calibrator_order get_calibrator_order() const override;
public:
    wizard(std::shared_ptr<const tracker_type> tracker);
};

} // ns calibrator_wizard
