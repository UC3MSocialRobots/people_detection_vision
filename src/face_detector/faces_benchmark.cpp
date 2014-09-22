/*!
  \file        faces_benchmark.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/8/12

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

 */

#include <vision_utils/img_path.h>
#include "time/timer.h"
#include "xml_images_interface/xml_images_interface.h"
#include <cv_conversion_float_uchar.h>
// detectors
#include "color_face_detector.h"
#include "texture_sensor.h"
#include "image_utils/opencv_face_detector.h"
// DAI
#include <dai/alldai.h>

class FaceDetectorsContainer {
public:
  // variables: 0:V (0/1), 1:F(0/1/2), 2:S:0..9, 3:T:0..9, 4:NN:0;1
  enum { // variables as defined in F.fg
    V=0, // [0,1]
    F = 1, // [0,1]
    S = 2, // [0,9]
    T = 3, // [0,9]
    NN = 4 // [0,1]
  };
  static const unsigned int nV = 2, nF = 3, nS = 10, nT = 10, nN = 2;

  FaceDetectorsContainer() {
    color_detector.load_default_training();
    haar_classifier = image_utils::create_face_classifier();
  }

  //////////////////////////////////////////////////////////////////////////////

  void run_on_image(const cv::Mat3b & img) {
    image_utils::detect_with_opencv
        (img, haar_classifier, small_img, rects);
    haar_detector_result = (rects.size() > 0);

    color_detector.test_apply_gaussian_fast3D(img, likelihood);
    color_detector_result = color_detector.test_get_likelihood(likelihood);

    cv::cvtColor(img, imgBW, CV_BGR2GRAY);
    texture_detector_result = texture_detector.correlation_ratio(imgBW);
  } // end run_on_image();

  //////////////////////////////////////////////////////////////////////////////

  inline int get_color_detector_result_clamp() const {
    return clamp((int) (10 * color_detector_result), 0, 9);
  }

  inline int get_texture_detector_result_clamp() const {
    return clamp((int) (10 * texture_detector_result), 0, 9);
  }

  inline int get_haar_detector_result() const {
    return haar_detector_result;
  }

  // protected:
  // color detector
  double color_detector_result;
  ColorFaceDetector color_detector;
  ColorFaceDetector::Likelihood likelihood;
  //! in [0..9]
  // texture detector
  double texture_detector_result;
  TextureSensor texture_detector;
  cv::Mat1b imgBW;
  //! in [0..9]

  // haar detector
  int haar_detector_result;
  cv::CascadeClassifier haar_classifier;
  cv::Mat3b frame;
  cv::Mat3b small_img;
  std::vector<cv::Rect> rects;
}; // end class FaceDetectorsContainer

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class FaceDetectorXmlGui : public XmlImagesInterface, public FaceDetectorsContainer {
public:
  //! \see XmlImagesReader::from_xml_node_custom()
  void from_xml_node_custom(const XmlDocument & doc, XmlDocument::Node* node) {
    // default values
    face_visible=0;
    face_frontal = 2;
    haar_detector_result = -1;
    color_detector_result = -1;
    texture_detector_result = -1;

    // try to load data from XML
    XmlDocument::Node* face_node = doc.get_node_at_direction(node, "face");
    if (face_node != NULL) {
      face_visible = doc.get_node_attribute(face_node, "visible", face_visible);
      face_frontal = doc.get_node_attribute(face_node, "frontal", face_frontal);
    }
    XmlDocument::Node* detectors_node = doc.get_node_at_direction(node, "detectors");
    if (detectors_node != NULL) {
      haar_detector_result = doc.get_node_attribute(detectors_node, "haar", haar_detector_result);
      color_detector_result = doc.get_node_attribute(detectors_node, "color", color_detector_result);
      texture_detector_result = doc.get_node_attribute(detectors_node, "texture", texture_detector_result);
    }
  }

  //////////////////////////////////////////////////////////////////////////////

  //! \see XmlImagesReader::to_xml_node_custom()
  void to_xml_node_custom(XmlDocument & doc, XmlDocument::Node* node) const {
    XmlDocument::Node* face_node = doc.add_node(node, "face", "");
    doc.set_node_attribute(face_node, "visible", face_visible);
    doc.set_node_attribute(face_node, "frontal", face_frontal);

    XmlDocument::Node* detectors_node = doc.add_node(node, "detectors", "");
    doc.set_node_attribute(detectors_node, "haar", haar_detector_result);
    doc.set_node_attribute(detectors_node, "color", color_detector_result);
    doc.set_node_attribute(detectors_node, "texture", texture_detector_result);
  }

  //////////////////////////////////////////////////////////////////////////////

  //! \see XmlImagesInterface::action_at_left_click()
  virtual void action_at_left_click(int x, int y) {
  }

  //////////////////////////////////////////////////////////////////////////////

  //! \see XmlImagesInterface::action_at_key_pressed()
  virtual void action_at_key_pressed(char c) {
    if (c == 'v') {
      if (face_visible) {
        face_visible = false;
        face_frontal = 2; // non applicable
      }
      else {
        face_visible = true;
        face_frontal = true; // almost all faces are frontal
      }
    }
    else if (c == 'f')
      face_frontal = !face_frontal;
    display_info();
  } //end action_at_key_pressed();

  //////////////////////////////////////////////////////////////////////////////

  //! \see XmlImagesInterface::refresh_window_custom()
  virtual void refresh_window_custom() {
    //cv::imshow("foo", *get_current_cv_img()); cv::waitKey(0);
    // run detectors if needed
    if (haar_detector_result == -1
        || color_detector_result == -1
        || texture_detector_result == -1) {
      run_on_image(*get_current_cv_img());
    }
    // display_info();
  }

  //////////////////////////////////////////////////////////////////////////////

  void display_info() {
    get_current_cv_img()->copyTo(_current_image_in_window);

    // display text
    printf("** '%s': \t", get_current_filename().c_str());
    printf("Visible:%i \t", face_visible);
    printf("frontal:%i \t", face_frontal);
    printf("color:%g \t", color_detector_result);
    printf("texture:%g \t", texture_detector_result);
    printf("haar:%i\n", haar_detector_result);
    int l1 = 10, lh = 10;
    cv::putText(_current_image_in_window,
                std::string("Visible:") + StringUtils::cast_to_string(face_visible),
                cv::Point(10, l1), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 0, 0));
    cv::putText(_current_image_in_window,
                std::string("frontal:") + StringUtils::cast_to_string(face_frontal),
                cv::Point(10, l1 + lh), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 0, 0));
    cv::putText(_current_image_in_window,
                std::string("color:") + StringUtils::cast_to_string(color_detector_result),
                cv::Point(10, l1 + 3 * lh), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 0, 0));
    cv::putText(_current_image_in_window,
                std::string("texture:") + StringUtils::cast_to_string(texture_detector_result),
                cv::Point(10, l1 + 4 * lh), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 0, 0));
    cv::putText(_current_image_in_window,
                std::string("haar:") + StringUtils::cast_to_string(haar_detector_result),
                cv::Point(10, l1 + 2 * lh), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 0, 0));
  }

  //////////////////////////////////////////////////////////////////////////////

  void generate_tab_file(const std::string tabfile = "/tmp/FaceDetectorXmlGui.tab") {
    // create file content according to
    // http://cs.ru.nl/~jorism/libDAI/doc/fileformats.html
    std::ostringstream ans;
    // The header line consists of the variable labels (corresponding to dai::Var::label())
    ans << V << "\t"  << F << "\t"  << S << "\t" << T << "\t" << NN << std::endl;
    // an empty line
    ans << std::endl;
    // data points
    for (int file_idx=0; file_idx < get_nb_files(); ++file_idx) {
      //  Each line (apart from the empty one) should have the same number of columns,
      //where columns are separated by one tab character.
      //  Each column corresponds to a variable.
      //  The header line consists of the variable labels (corresponding to dai::Var::label()).
      //  The other lines are observed joint states of the variables, i.e., each
      //line corresponds to a joint observation of the variables, and each column of
      //a line contains the state of the variable associated with that column.
      //  Missing data is handled simply by having two consecutive tab characters,
      //without any characters in between.
      printf("file:'%s'\n", get_current_filename().c_str());
      if (!is_current_file_valid()) {
        go_to_next_file(false);
        continue;
      }
      ans << face_visible << "\t";
      ans << face_frontal << "\t";
      ans << get_color_detector_result_clamp() << "\t";
      ans << get_texture_detector_result_clamp() << "\t";
      ans << haar_detector_result << std::endl; // end line
      go_to_next_file(false);
    } // end for (file_idx)
    StringUtils::save_file(tabfile, ans.str());
    printf("Saved tabfile '%s'.\n", tabfile.c_str());
  } // end generate_tab_file();

  //////////////////////////////////////////////////////////////////////////////

  inline int is_face_visible() const {
    return face_visible;
  }
  inline int is_face_frontal() const {
    return face_frontal;
  }

protected:
  int face_visible;
  int face_frontal;
}; // end class FaceDetectorXmlGui

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class MultiModalFaceDetector : public FaceDetectorsContainer {
public:
  MultiModalFaceDetector()
    : V(FaceDetectorXmlGui::V, nV),
      F(FaceDetectorXmlGui::F, nF),
      S(FaceDetectorXmlGui::S, nS),
      T(FaceDetectorXmlGui::T, nT),
      N(FaceDetectorXmlGui::NN, nN) {
  }

  //////////////////////////////////////////////////////////////////////////////

  void create_fg(const std::string & fg_outfilename) {
    dai::Factor P_V(V);
    dai::Factor P_F_given_V(dai::VarSet(F, V));
    dai::Factor P_S_given_V(dai::VarSet(S, V));
    dai::Factor P_T_given_V(dai::VarSet(T, V));
    //  dai::Factor P_N_given_VF(dai::VarSet(N, V, F));
    dai::VarSet setVFN;
    setVFN |= N;
    setVFN |= V;
    setVFN |= F;
    dai::Factor P_N_given_VF(setVFN);

    // Build factor graph consisting of those four factors
    std::vector<dai::Factor> SprinklerFactors;
    SprinklerFactors.push_back(P_V);
    SprinklerFactors.push_back(P_F_given_V);
    SprinklerFactors.push_back(P_S_given_V);
    SprinklerFactors.push_back(P_T_given_V);
    SprinklerFactors.push_back(P_N_given_VF);
    dai::FactorGraph network(SprinklerFactors);

    // Write factorgraph to a file: will generate five blocks
    // factor: P(V)
    // factor: P(F|V)
    // factor: P(S|V)
    // factor: P(T|V)
    // factor: P(N|V, F)
    network.WriteToFile(fg_outfilename.c_str());
    std::cout << "Sprinkler network written to " << fg_outfilename << std::endl;
  }

  //////////////////////////////////////////////////////////////////////////////

  void create_em_optimized_fg(const std::string & fg_infilename,
                              const std::string & tab_filename,
                              const std::string & em_filename,
                              const std::string & fg_outfilename) {
    // Read the factorgraph from the file
    dai::FactorGraph network;
    network.ReadFromFile(fg_infilename.c_str());

    // Prepare junction-tree object for doing exact inference for E-step
    dai::PropertySet infprops;
    infprops.set("verbose", (size_t)1);
    infprops.set("updates", std::string("HUGIN"));
    dai::InfAlg* inf = dai::newInfAlg("JTREE", network, infprops);
    inf->init();

    // Read sample from file
    dai::Evidence e;
    std::ifstream estream(tab_filename.c_str());
    e.addEvidenceTabFile(estream, network);
    std::cout << "Number of samples: " << e.nrSamples() << std::endl;

    // Read EM specification
    std::ifstream emstream(em_filename.c_str());
    dai::EMAlg em(e, *inf, emstream);

    // Iterate EM until convergence
    while(!em.hasSatisfiedTermConditions()) {
      dai::Real l = em.iterate();
      std::cout << "Iteration " << em.Iterations()
                << " likelihood: " << l << std::endl;
    }

    // Output true factor graph
    std::cout << std::endl << "True factor graph:" << std::endl << "##################" << std::endl;
    std::cout.precision(12);
    std::cout << network;

    // Output learned factor graph
    std::cout << std::endl << "Learned factor graph:" << std::endl << "#####################" << std::endl;
    std::cout.precision(12);
    std::cout << inf->fg();

    // https://bitbucket.org/jlebar/chess/src/7dfde75be5870e339c2f8a65871af9235e105d9a/libdai-analysis/create_em_optimized_fg.cpp?at=defaulthttps://bitbucket.org/jlebar/chess/src/7dfde75be5870e339c2f8a65871af9235e105d9a/libdai-analysis/test_model.cpp?at=default
    //((dai::FactorGraph*) (&inf->fg()))->WriteToFile(fg_outfilename.c_str());
    StringUtils::save_file(fg_outfilename, inf->fg().toString());
    std::cout << "Optimized network written to " << fg_outfilename << std::endl;

    // Clean up
    delete inf;
  } // end create_em_optimized_fg();

  //////////////////////////////////////////////////////////////////////////////

  void load_optimized_fg(const std::string & fg_infilename) {
    network.ReadFromFile(fg_infilename.c_str());
    // Set up inference algorithm
    dai::PropertySet infprops;
    infprops.set( "verbose", (size_t)0 );
    instantiator = dai::newInfAlg( "EXACT", network, infprops );
    instantiator->init();
    instantiator->run();
    std::cout << "Prior Beliefs: \n" << instantiator->beliefs() << std::endl;
  }

  //////////////////////////////////////////////////////////////////////////////

  void display_learned_cpt() {
    // Calculate joint probability of all four variables
    dai::Factor P;
    for(size_t I=0; I < network.nrFactors(); I++)
      P *= network.factor(I);
    // P.normalize();  // Not necessary: a Bayesian network is already normalized by definition

    // Calculate some probabilities
    dai::Real PV1 = P.marginal(V)[1];
    std::cout << "P.marginal(V) = " << P.marginal(V) << std::endl;
    std::cout << "P(V=1) = " << PV1 << std::endl;
    std::cout << "P.marginal(VarSet(S, V)) = " << P.marginal(dai::VarSet(S, V)) << std::endl;
    std::cout << "P(S=1 | V=1) = " << P.marginal(dai::VarSet(S, V))[3] / PV1 << std::endl;
    //  cout << "P(R=1 | V=1) = " << P.marginal(VarSet(R, W))[3] / denom << endl;

    //dai::Factor CPT = P.marginal(setVFN); // P(N, V, F)
    std::cout << "P(V=0)" << P.marginal(V)[0] << std::endl;
    std::cout << "P(V=1)" << P.marginal(V)[1] << std::endl;
    std::cout << "P(F=0)" << P.marginal(F)[0] << std::endl;
    std::cout << "P(F=1)" << P.marginal(F)[1] << std::endl;
    std::cout << "P(F=2)" << P.marginal(F)[2] << std::endl;

    // local CPT
    // P (N=0 | V=0, F=0) = P (N=0, V=0, F=0) / (P(V=0) * P(F=0))
    // The most difficult part is getting the indexing right.
    // The convention that is used is that the left-most variables cycle through
    // their values the fastest (similar to MatLab indexing of multidimensional
    // arrays).
    dai::VarSet setVFN;
    setVFN |= V;
    setVFN |= F;
    setVFN |= N;
    dai::Factor CPT = network.factor(network.findFactor(setVFN)); // P(N, V, F)
    std::cout << "CPT" << CPT << std::endl;
    std::cout << "V\tF\tP(N=0)\tP(N=1)" << std::endl;
    for (unsigned int F_idx = 0; F_idx < nF; ++F_idx) {
      for (unsigned int V_idx = 0; V_idx < nV; ++V_idx) {
        double probaN0 = CPT[V_idx + F_idx * nV]
            ;// / (P.marginal(V)[V_idx] * P.marginal(F)[F_idx]);
        double probaN1 = CPT[V_idx + F_idx * nV + nF * nV]
            ;// / (P.marginal(V)[V_idx] * P.marginal(F)[F_idx]);
        std::cout << V_idx << "\t" << F_idx
                  << "\t" << probaN0 << "\t" << 1- probaN0
                  << " (" << probaN1 << ")"
                  << std::endl;
      } // end loop V_idx
    } // end loop F_idx
  }

  ////////////////////////////////////////////////////////////////////////////////

  //! \return P(V = 1 | N, S, T)
  double inference(unsigned int N_val, unsigned int S_val, unsigned int T_val) {
    dai::InfAlg* instantiator_clone = instantiator->clone();
    instantiator_clone->clamp(N.label(), N_val);
    instantiator_clone->clamp(S.label(), S_val);
    instantiator_clone->clamp(T.label(), T_val);
    instantiator_clone->init();
    instantiator_clone->run();
#if 0
    std::cout << "P(V=" << 0
              << " | S=" << S_val << ", T=" << T_val << ", N=" << N_val << ") = "
              << instantiator_clone->beliefV(V.label())
              << std::endl;
#endif
    return instantiator_clone->beliefV(V.label())[1];
  } // end display_inference_NST();

  //////////////////////////////////////////////////////////////////////////////

  void display_inference_NST() {
    // direct lookup - VFSTN
    for (unsigned int N_idx = 0; N_idx < nN; ++N_idx) {
      for (unsigned int S_idx = 0; S_idx < nS; ++S_idx) {
        for (unsigned int T_idx = 0; T_idx < nT; ++T_idx) {
          inference(N_idx, S_idx, T_idx);
        } // end loop T_idx
      } // end loop S_idx
    } // end loop N_idx
  } // end display_inference_NST();

  //////////////////////////////////////////////////////////////////////////////

  bool predict(const cv::Mat3b & img) {
    run_on_image(img);
    double p = inference(haar_detector_result,
                         get_color_detector_result_clamp(),
                         get_texture_detector_result_clamp());
    return (p > .5); // p = P(V = 1 | N, S, T)
  }

  //////////////////////////////////////////////////////////////////////////////
protected:
  dai::FactorGraph network;
  // variables: 0:V (0/1), 1:F(0/1/2), 2:S:0..9, 3:T:0..9, 4:NN:0;1
  dai::Var V, F, S, T, N;
  dai::InfAlg* instantiator;
}; // end class MultiModalFaceDetector


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void interface() {
  FaceDetectorXmlGui interf;
  interf.from_xml_file(IMG_DIR "faces_color/", "faces.xml", true);
}

////////////////////////////////////////////////////////////////////////////////

void generate_tab_file() {
  FaceDetectorXmlGui interf;
  interf.from_xml_file(IMG_DIR "faces_color/", "faces.xml", false);
  interf.generate_tab_file(IMG_DIR "faces_color/F.tab");
}

////////////////////////////////////////////////////////////////////////////////

void test_on_cam(int device = 0) {
  cv::VideoCapture capture(device);
  Timer timer;
  MultiModalFaceDetector detector;
  detector.load_optimized_fg(IMG_DIR "faces_color/F_optimized.fg");
  timer.printTime("load_optimized_fg()");

  int w1 = 20, h1 = 40;
  cv::Mat1b img_bw;
  cv::Mat1f img_corr;
  cv::Mat3b float_out_color;
  cv::Mat1b likelihood_greyscale;
  cv::Mat3b test_img_clone;

  while(capture.isOpened()) {
    cv::Mat3b test_img;
    capture >> test_img;
    timer.reset();
    bool has_face = detector.predict(test_img);
    timer.printTime("detector.predict()");

    ColorFaceDetector::likelihood2greyscale(detector.likelihood, likelihood_greyscale);
    cv::imshow("ColorFaceDetector", likelihood_greyscale);

    cv::cvtColor(test_img, img_bw, CV_BGR2GRAY);
    detector.texture_detector.mosaic_img(img_bw, img_corr, w1, h1);
    image_utils::depth_image_to_vizualisation_color_image
        (img_corr, float_out_color, image_utils::FULL_RGB_STRETCHED);
    cv::imshow("TextureSensor", float_out_color);

    test_img.copyTo(test_img_clone);
    for (unsigned int rect_idx = 0; rect_idx < detector.rects.size(); ++rect_idx)
      cv::rectangle(test_img_clone, detector.rects[rect_idx], CV_RGB(0, 255, 0), 3);
    cv::imshow("HaarClasifier", test_img_clone);

    if (has_face) {
      cv::putText(test_img, "FACE!", cv::Point(50, 50), cv::FONT_HERSHEY_PLAIN,
                  2, CV_RGB(0, 0, 0), 3);
      cv::putText(test_img, "FACE!", cv::Point(50, 50), cv::FONT_HERSHEY_PLAIN,
                  2, CV_RGB(0, 255, 0), 2);
    }
    cv::imshow("MultiModalFaceDetector", test_img);
    int c = cv::waitKey(5);
    if  (c == 27)
      break;
  } // end while(capture.isOpened())
} // end test_on_cam();

////////////////////////////////////////////////////////////////////////////////

struct Error {
  Error() {
    true_positive = false_positive = true_negative = false_negative = 0;
  }

  inline void update(unsigned int expected, unsigned int obtained) {
    if (expected == obtained) {
      if (expected)
        ++true_positive;
      else
        ++true_negative;
    }
    else {
      if (expected)
        ++false_negative;
      else
        ++false_positive;
    }
  } // end update

  inline std::string to_string() const {
    std::ostringstream out;
    out << true_positive << "-truepos(hit), ";
    out << true_negative << "-trueneg(correct rejection),";
    out << false_positive << "-falsepos(false alarm),";
    out << false_negative << "-falseneg(miss),";
    out << " hitrate(recall):" << std::setprecision(3)
        << 1. * true_positive / (true_positive + false_negative);
    out << " accuracy:" << std::setprecision(3)
        << 1. * (true_positive + true_negative)
           / (true_positive + false_positive + true_negative + false_negative);
    return out.str();
  }
  unsigned int true_positive, true_negative, false_positive, false_negative;
}; // en class Error

void benchmark(const std::string & path_to_xml_file,
               const std::string & xml_filename) {
  FaceDetectorXmlGui interf;
  interf.from_xml_file(path_to_xml_file, xml_filename, false);
  MultiModalFaceDetector detector;
  detector.load_optimized_fg(IMG_DIR "faces_color/F_optimized.fg");

  int nFiles = 0;
  Error ecolor, etexture, ehaar, emultimodal;

  std::cout << "Truth \tColor \tTexture \tHaar \tMultimodal" << std::endl;
  for (int file_idx=0; file_idx < interf.get_nb_files(); ++file_idx) {
    if (!interf.is_current_file_valid()) {
      interf.go_to_next_file(false);
      continue;
    }
    ++nFiles;

    // run each detector
    bool ground_truth = interf.is_face_visible();
    bool color_res = (detector.get_color_detector_result_clamp() >= 5);
    bool texture_res = (detector.get_texture_detector_result_clamp() >= 5);
    bool haar_res = detector.get_haar_detector_result();
    bool multimodal_res = detector.predict(*interf.get_current_cv_img());

    ecolor.update(ground_truth, color_res);
    etexture.update(ground_truth, texture_res);
    ehaar.update(ground_truth, haar_res);
    emultimodal.update(ground_truth, multimodal_res);
    //    std::cout << ground_truth << "\t\t"
    //              << color_res << "\t"
    //              << texture_res << "\t"
    //              << haar_res << "\t"
    //              << multimodal_res << std::endl;
    interf.go_to_next_file(false);

    if (nFiles % 100 == 0) {
      interf.display_info();
      std::cout << nFiles << "files:" << std::endl
                << "ecolor:" << ecolor.to_string() << std::endl
                << "etexture:" << etexture.to_string() << std::endl
                << "ehaar:" << ehaar.to_string() << std::endl
                << "emultimodal:" << emultimodal.to_string() << std::endl;
    }

  } // end for (file_idx)

  std::cout << nFiles << "files:" << std::endl
            << "ecolor:" << ecolor.to_string() << std::endl
            << "etexture:" << etexture.to_string() << std::endl
            << "ehaar:" << ehaar.to_string() << std::endl
            << "emultimodal:" << emultimodal.to_string() << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  int idx = 1;
  if (argc < 2) {
    printf("%i: interface()\n", idx++);
    printf("%i: generate_tab_file();\n", idx++);
    printf("%i: create_fg()\n", idx++);
    printf("%i: create_em_optimized_fg()\n", idx++);
    printf("%i: display_learned_cpt()\n", idx++);
    printf("%i: display_inference_NST()\n", idx++);
    printf("%i: test_on_cam();\n", idx++);
    printf("%i: benchmark(IMG_DIR faces_color/, faces.xml);\n", idx++);
    printf("%i: benchmark(IMG_DIR faces_color/, faces_benchmark.xml);\n", idx++);
    return -1;
  }
  int choice = 1;
  choice = atoi(argv[1]);
  //std::cin >> choice;

  idx = 1;
  if (choice == idx++)
    interface();
  if (choice == idx++)
    generate_tab_file();
  else if (choice == idx++) {
    MultiModalFaceDetector detector;
    detector.create_fg(IMG_DIR "faces_color/F.fg");
  }
  else if (choice == idx++) {
    MultiModalFaceDetector detector;
    detector.create_em_optimized_fg(IMG_DIR "faces_color/F.fg",
                                    IMG_DIR "faces_color/F.tab",
                                    IMG_DIR "faces_color/F_stripped.em",
                                    IMG_DIR "faces_color/F_optimized.fg");
  }
  else if (choice == idx++) {
    MultiModalFaceDetector detector;
    detector.load_optimized_fg(IMG_DIR "faces_color/F_optimized.fg");
    detector.display_learned_cpt();
  }
  else if (choice == idx++) {
    MultiModalFaceDetector detector;
    detector.load_optimized_fg(IMG_DIR "faces_color/F_optimized.fg");
    detector.display_inference_NST();
  }
  else if (choice == idx++)
    test_on_cam(argc >= 3 ? atoi(argv[2]) : 0);
  else if (choice == idx++)
    benchmark(IMG_DIR "faces_color/", "faces.xml");
  else if (choice == idx++)
    benchmark(IMG_DIR "faces_color/", "faces_benchmark.xml");
  return 0;
}
