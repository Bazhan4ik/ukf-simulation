

class SigmaPoints {
public:
    int n;
    double alpha;
    double kappa;
    double lambda;
    double beta;

    int nps;
    double scale;


    std::vector<double> weights;
    std::vector<double> Pweights;

    SigmaPoints() {
        n = 6;
        alpha = 0.001;
        kappa = 5.0;
        beta = 2.0;
        lambda = alpha * alpha * (n + kappa) - n;
        nps = 1 + 2 * n;
        scale = n + lambda;


        weights.emplace_back(lambda / (n + lambda));
        Pweights.emplace_back(
            lambda / (n + lambda) + 1 - alpha * alpha + beta
        );
        for(int i = 0; i < nps; i++) {
            weights.emplace_back(0.5 / (n + lambda));
            Pweights.emplace_back(0.5 / (n + lambda));
        }
    }

    void outputParameters() {
        std::cout << "n: " << n << std::endl;
        std::cout << "lambda: " << lambda << std::endl;
        std::cout << "alpha: " << alpha << std::endl;
        std::cout << "kappa: " << kappa << std::endl;
        std::cout << "beta: " << beta << std::endl;
        std::cout << "sigma points amount: " << nps << std::endl;
        std::cout << "sigma points P scale: " << scale << std::endl;
        std::cout << "\n\n" << std::endl;
    }

    double getw(int i) {
        return weights.at(i);
    }
    double getPw(int i) {
        return Pweights.at(i);
    }
};