// Independent strategy (log1p(lux) input, depth 2)
double score_exposure_sep(double * input) {
    double var0;
    if (input[0] <= 5.942264556884766) {
        if (input[0] <= 4.339601039886475) {
            var0 = 33231.95348837209;
        } else {
            var0 = 23458.764705882353;
        }
    } else {
        if (input[0] <= 7.899871110916138) {
            var0 = 12304.470588235294;
        } else {
            var0 = 1899.635838150289;
        }
    }
    return var0;
}

double score_gain_sep(double * input) {
    double var0;
    if (input[0] <= 2.905330777168274) {
        if (input[0] <= 2.169829487800598) {
            var0 = 35638.818181818184;
        } else {
            var0 = 20841.545454545456;
        }
    } else {
        if (input[0] <= 4.339601039886475) {
            var0 = 10708.52380952381;
        } else {
            var0 = 189.6183574879227;
        }
    }
    return var0;
}
