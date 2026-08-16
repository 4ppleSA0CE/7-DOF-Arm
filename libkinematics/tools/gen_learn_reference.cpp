// Generates learn/test/reference.json: the oracle the JavaScript lecture engine
// in learn/engine/ is checked against.
//
// The JS is an independent second implementation of everything in libkinematics.
// "Independent" is only worth something if the numbers it is compared with come
// from here, so this tool writes them out and nothing else does.
//
//   cmake --build build --target gen_learn_reference
//   ./build/libkinematics/gen_learn_reference > learn/test/reference.json
//   node learn/test/engine.test.mjs
//
// The eight configurations below are FIXTURE INPUTS, pinned in the source rather
// than drawn from an RNG: regenerating has to produce a file that can be diffed
// against the previous one, which it cannot do if the inputs move. Case 0 is the
// home pose q = 0, which is an exact singularity (three rows of J_b vanish) and
// is therefore the case that exercises the rank-deficient branch of the
// pseudoinverse. The rest are random draws inside the joint limits, three of them
// carrying an external wrench.
//
// Twist order [omega; v], wrench [m; f]. SI units.
#include <cstdio>
#include <iostream>
#include <vector>

#include "libkinematics/dynamics.hpp"
#include "libkinematics/fk.hpp"
#include "libkinematics/jacobian.hpp"
#include "libkinematics/null_space.hpp"
#include "libkinematics/robot.hpp"

namespace {

struct Case {
  std::vector<double> q, qd, qdd, f_ext;
};

// clang-format off
const std::vector<Case> kCases = {
  {{0,0,0,0,0,0,0},
   {0,0,0,0,0,0,0},
   {0,0,0,0,0,0,0},
   {0,0,0,0,0,0}},

  {{2.4514143620251243, 4.657096724473362, -0.2392240622486272, 1.3350380958930925,
    -1.3100291281421932, 3.6517796559843667, 2.135775660537489},
   {-0.7385854118893036, 0.06415583312871176, 0.8474763104008405, 0.7846874532977655,
    -0.20251192799469253, 0.8158497949176526, -0.89902408620171},
   {-0.9204810069948813, 0.9126199491301294, -0.2521576454169552, -0.946420544776457,
    0.6145774160040094, 0.1127946040929182, 0.6124696902197204},
   {0,0,0,0,0,0}},

  {{2.7068948318903936, 1.420657588081059, 1.4903857438016221, 4.686057723689285,
    2.509918223359127, 5.048016626971301, -2.496033148894308},
   {-0.2719408290145924, 0.6653727957651434, 0.19353928686623578, 0.39696228774281694,
    0.6637986081148253, 0.312239172426964, -0.5249430840749527},
   {0.381895842647918, -0.363292536701069, -0.2725863120654066, 0.6437578592001101,
    0.8199154317966919, 0.6239791274513535, -0.24130957457226576},
   {0,0,0,0,0,0}},

  {{1.2518370510974934, 3.7853403224076208, 1.215515992980923, 4.908104328082934,
    -3.1020268497438517, 2.642860128592686, -1.5743350358292056},
   {-0.03010950326175965, 0.5185799894240246, 0.8265182441929602, -0.2335895282361592,
    -0.12488514364222836, -0.013501067250687004, -0.30572951942892},
   {-0.1695555340410443, 0.11504578611784289, -0.950774351353082, -0.4024579492732089,
    0.47587456064747036, -0.9719206979912253, 0.5933733552547145},
   {0.8767987285683947, -0.7989558609547831, 0.4708053086360897, 0.9528742380438404,
    0.41177151146043856, 0.9035293487707292}},

  {{-2.1857140138832363, 2.015109352545554, -1.1651159225318863, 1.0411829092657527,
    -0.46096177151085965, 2.954220264739162, -1.0697169845792236},
   {0.36118179502982506, -0.12406472717070816, -0.2985362639527859, 0.862920653166154,
    -0.45863808501222025, -0.8335284135819686, -0.6157897773284378},
   {-0.7480822411121757, -0.78066477742478, 0.5222857286679097, -0.34784185313875726,
    0.576219540621594, -0.859246463115967, 0.6757305626343544},
   {0,0,0,0,0,0}},

  {{-0.4933336551526975, 4.719595762986721, -0.5701559007541719, 4.5905863669036675,
    2.968876874210042, 4.098758172085499, 2.7049288785235692},
   {-0.6682755014865549, 0.830900936366401, -0.5557433429730607, 0.4746910116700882,
    -0.18008112262987264, -0.46658090232194904, 0.7493772432157755},
   {-0.5783304665555793, 0.3631829321627753, 0.6706000194054604, 0.7174044314790085,
    0.19402793784603767, 0.5262661189274509, -0.7676732252103708},
   {0.9345784647071163, -0.3940251155963328, 0.21803164895777094, -0.6644859100375515,
    0.7168518942574815, 0.4527845755219959}},

  {{-2.335886277804952, 2.0422014087358233, 2.014663109774041, 3.1625546737756722,
    -0.08466342908233938, 2.9174293642428175, -1.7012829411051495},
   {-0.9480144528337766, 0.42295914722354877, 0.5820113654199641, -0.13860258583572227,
    -0.19921043703198527, -0.5321531375880331, -0.4970977842500519},
   {0.464231143110152, 0.3548694298711923, 0.2115197270270912, -0.7983170317395107,
    0.17434280221004284, 0.2684233336160382, 0.9571411226658375},
   {-0.10679465760793416, -0.8382244155139479, -0.7604050639027904, -0.07444352889388584,
    0.5569716974150221, -0.2030112898524098}},

  {{2.7036566079701774, 2.8687959467621362, -2.0733405546571775, 5.540746075118404,
    -0.2470530049771039, 3.462682141512163, 0.975009641970053},
   {-0.7923380207798079, 0.24640099928453507, -0.3342388941298017, -0.8707791755596321,
    -0.25550964579961455, -0.7318933365768472, 0.42134610379123383},
   {-0.7481895424228961, 0.21560226168326024, -0.7018390887148405, -0.7658806590264262,
    0.9039845625195109, -0.8847198197612955, -0.1905231417223111},
   {0,0,0,0,0,0}},
};
// clang-format on

// 17 significant digits round-trips a double exactly through JSON.
void num(double x) { std::printf("%.17g", x); }

void vec(const Eigen::VectorXd& v) {
  std::printf("[");
  for (int i = 0; i < v.size(); ++i) {
    if (i) std::printf(", ");
    num(v(i));
  }
  std::printf("]");
}

void mat(const Eigen::MatrixXd& M) {
  std::printf("[");
  for (int i = 0; i < M.rows(); ++i) {
    if (i) std::printf(", ");
    std::printf("[");
    for (int j = 0; j < M.cols(); ++j) {
      if (j) std::printf(", ");
      num(M(i, j));
    }
    std::printf("]");
  }
  std::printf("]");
}

Eigen::VectorXd toEigen(const std::vector<double>& v) {
  Eigen::VectorXd x(static_cast<int>(v.size()));
  for (std::size_t i = 0; i < v.size(); ++i) x(static_cast<int>(i)) = v[i];
  return x;
}

}  // namespace

int main() {
  klib::Robot robot = klib::load_robot_yaml(KINOVA_SCREWS_YAML);
  robot = klib::load_dynamics_yaml(std::move(robot), KINOVA_DYNAMICS_YAML);
  const klib::ForwardKinematics fk(robot);

  std::printf("{\n");
  std::printf(
      "  \"note\": \"generated by libkinematics (C++); see learn/test/README\",\n");
  std::printf("  \"cases\": [\n");

  for (std::size_t c = 0; c < kCases.size(); ++c) {
    const Eigen::VectorXd q = toEigen(kCases[c].q);
    const Eigen::VectorXd qd = toEigen(kCases[c].qd);
    const Eigen::VectorXd qdd = toEigen(kCases[c].qdd);
    klib::Wrench f_ext;
    for (int i = 0; i < 6; ++i) f_ext(i) = kCases[c].f_ext[i];

    const klib::Jacobian Jb = klib::body_jacobian(robot, q);

    std::printf("    {\n");
    std::printf("      \"q\": ");        vec(q);                              std::printf(",\n");
    std::printf("      \"qd\": ");       vec(qd);                             std::printf(",\n");
    std::printf("      \"qdd\": ");      vec(qdd);                            std::printf(",\n");
    std::printf("      \"f_ext\": ");    vec(f_ext);                          std::printf(",\n");
    std::printf("      \"T\": ");        mat(fk.body_pose(q).matrix());       std::printf(",\n");

    std::printf("      \"link_T\": [");
    const auto links = fk.link_poses(q);
    for (std::size_t i = 0; i < links.size(); ++i) {
      if (i) std::printf(", ");
      mat(links[i].matrix());
    }
    std::printf("],\n");

    std::printf("      \"Jb\": ");   mat(Jb);                                        std::printf(",\n");
    std::printf("      \"Js\": ");   mat(klib::space_jacobian(robot, q));            std::printf(",\n");
    std::printf("      \"manip\": "); num(klib::manipulability(robot, q));           std::printf(",\n");
    std::printf("      \"N\": ");    mat(klib::null_space_projector(Jb));            std::printf(",\n");
    std::printf("      \"M\": ");    mat(klib::mass_matrix(robot, q));                std::printf(",\n");
    std::printf("      \"g\": ");    vec(klib::gravity_term(robot, q));               std::printf(",\n");
    std::printf("      \"Cqd\": ");  vec(klib::coriolis_term(robot, q, qd));          std::printf(",\n");
    std::printf("      \"tau\": ");  vec(klib::inverse_dynamics(robot, q, qd, qdd, f_ext)); std::printf(",\n");
    std::printf("      \"Cmat\": "); mat(klib::coriolis_matrix(robot, q, qd));        std::printf("\n");
    std::printf("    }%s\n", c + 1 == kCases.size() ? "" : ",");
  }

  std::printf("  ]\n}\n");
  return 0;
}
