/*
 * Copyright 2012 by Alison Leonard alison@makerbot.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#ifndef FEEDRATE_TABLE_HH_
#define FEEDRATE_TABLE_HH_

static int16_t ifeedrate_table[] PROGMEM = { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  31250,  30303,  29411,  28571,  27777,  27027,  26315,  25641,  25000,  24390,  23809,  23255,  22727,  22222,  21739,  21276,  20833,  20408,  20000,  19607,  19230,  18867,  18518,  18181,  17857,  17543,  17241,  16949,  16666,  16393,  16129,  15873,  15625,  15384,  15151,  14925,  14705,  14492,  14285,  14084,  13888,  13698,  13513,  13333,  13157,  12987,  12820,  12658,  12500,  12345,  12195,  12048,  11904,  11764,  11627,  11494,  11363,  11235,  11111,  10989,  10869,  10752,  10638,  10526,  10416,  10309,  10204,  10101,  10000,  9900,  9803,  9708,  9615,  9523,  9433,  9345,  9259,  9174,  9090,  9009,  8928,  8849,  8771,  8695,  8620,  8547,  8474,  8403,  8333,  8264,  8196,  8130,  8064,  8000,  7936,  7874,  7812,  7751,  7692,  7633,  7575,  7518,  7462,  7407,  7352,  7299,  7246,  7194,  7142,  7092,  7042,  6993,  6944,  6896,  6849,  6802,  6756,  6711,  6666,  6622,  6578,  6535,  6493,  6451,  6410,  6369,  6329,  6289,  6250,  6211,  6172,  6134,  6097,  6060,  6024,  5988,  5952,  5917,  5882,  5847,  5813,  5780,  5747,  5714,  5681,  5649,  5617,  5586,  5555,  5524,  5494,  5464,  5434,  5405,  5376,  5347,  5319,  5291,  5263,  5235,  5208,  5181,  5154,  5128,  5102,  5076,  5050,  5025,  5000,  4975,  4950,  4926,  4901,  4878,  4854,  4830,  4807,  4784,  4761,  4739,  4716,  4694,  4672,  4651,  4629,  4608,  4587,  4566,  4545,  4524,  4504,  4484,  4464,  4444,  4424,  4405,  4385,  4366,  4347,  4329,  4310,  4291,  4273,  4255,  4237,  4219,  4201,  4184,  4166,  4149,  4132,  4115,  4098,  4081,  4065,  4048,  4032,  4016,  4000,  3984,  3968,  3952,  3937,  3921,  3906,  3891,  3875,  3861,  3846,  3831,  3816,  3802,  3787,  3773,  3759,  3745,  3731,  3717,  3703,  3690,  3676,  3663,  3649,  3636,  3623,  3610,  3597,  3584,  3571,  3558,  3546,  3533,  3521,  3508,  3496,  3484,  3472,  3460,  3448,  3436,  3424,  3412,  3401,  3389,  3378,  3367,  3355,  3344,  3333,  3322,  3311,  3300,  3289,  3278,  3267,  3257,  3246,  3236,  3225,  3215,  3205,  3194,  3184,  3174,  3164,  3154,  3144,  3134,  3125,  3115,  3105,  3095,  3086,  3076,  3067,  3058,  3048,  3039,  3030,  3021,  3012,  3003,  2994,  2985,  2976,  2967,  2958,  2949,  2941,  2932,  2923,  2915,  2906,  2898,  2890,  2881,  2873,  2865,  2857,  2849,  2840,  2832,  2824,  2816,  2808,  2801,  2793,  2785,  2777,  2770,  2762,  2754,  2747,  2739,  2732,  2724,  2717,  2710,  2702,  2695,  2688,  2680,  2673,  2666,  2659,  2652,  2645,  2638,  2631,  2624,  2617,  2610,  2604,  2577,  2551,  2525,  2500,  2475,  2450,  2427,  2403,  2380,  2358,  2336,  2314,  2293,  2272,  2252,  2232,  2212,  2192,  2173,  2155,  2136,  2118,  2100,  2083,  2066,  2049,  2032,  2016,  2000,  1984,  1968,  1953,  1937,  1923,  1908,  1893,  1879,  1865,  1851,  1838,  1824,  1811,  1798,  1785,  1773,  1760,  1748,  1736,  1724,  1712,  1700,  1689,  1677,  1666,  1655,  1644,  1633,  1623,  1612,  1602,  1592,  1582,  1572,  1562,  1552,  1543,  1533,  1524,  1515,  1506,  1497,  1488,  1479,  1470,  1461,  1453,  1445,  1436,  1428,  1420,  1412,  1404,  1396,  1388,  1381,  1373,  1366,  1358,  1351,  1344,  1336,  1329,  1322,  1315,  1308,  1302,  1295,  1288,  1282,  1275,  1269,  1262,  1256,  1250,  1243,  1237,  1231,  1225,  1219,  1213,  1207,  1201,  1196,  1190,  1184,  1179,  1173,  1168,  1162,  1157,  1152,  1146,  1141,  1136,  1131,  1126,  1121,  1116,  1111,  1106,  1101,  1096,  1091,  1086,  1082,  1077,  1072,  1068,  1063,  1059,  1054,  1050,  1046,  1041,  1037,  1033,  1028,  1024,  1020,  1016,  1012,  1008,  1004,  1000,  996,  992,  988,  984,  980,  976,  961,  946,  932,  919,  905,  892,  880,  868,  856,  844,  833,  822,  811,  801,  791,  781,  771,  762,  753,  744,  735,  726,  718,  710,  702,  694,  686,  679,  672,  664,  657,  651,  644,  637,  631,  625,  618,  612,  606,  600,  595,  589,  584,  578,  573,  568,  563,  558,  553,  548,  543,  538,  534,  529,  525,  520,  516,  512,  508,  504,  500,  496,  492,  488,  484,  480,  477,  473,  469,  466,  462,  459,  456,  452,  449,  446,  443,  440,  437,  434,  431,  428,  425,  422,  419,  416,  413,  411,  408,  405,  403,  400,  398,  395,  393,  390,  388,  385,  383,  381,  378,  376,  374,  372,  369,  367,  365,  363,  361,  359,  357,  355,  353,  351,  349,  347,  345,  343,  341,  339,  337,  336,  334,  332,  330,  328,  327,  325,  323,  322,  320,  318,  317,  315,  314,  312,  310,  309,  307,  306,  304,  303,  301,  300,  299,  297,  296,  294,  293,  292,  290,  289,  288,  286,  285,  284,  282,  281,  280,  279,  277,  276,  275,  274,  272,  271,  270,  269,  268,  267,  265,  264,  263,  262,  261,  260,  259,  258,  257,  256,  255,  254,  253,  252,  251,  250,  249,  248,  247,  246,  245,  244,  229,  217,  205,  195,  186,  177,  169,  162,  156,  150,  144,  139,  134,  130,  126,  122,  118,  114,  111,  108,  105,  102,  100,  97,  95,  93,  90,  88,  86,  84,  83,  81,  79,  78,  76,  75,  73,  72,  71,  69,  68,  67,  66,  65,  64,  63,  62,  61,  60,  59,  58,  57,  56,  55,  55,  54,  53,  52,  52,  51,  50,  50,  49,  48,  48,  47,  47,  46,  45,  45,  44,  44,  43,  43,  42,  42,  42,  41,  41,  40,  40,  39,  39,  39,  38,  38,  37,  37,  37,  36,  36,  36,  35,  35,  35,  34,  34,  34,  33,  33,  33,  33,  32,  32,  32,  32,  31,  31,  31,  31,  30,  30,  30,  30,  29,  29,  29,  29,  28,  28,  28,  28,  28,  27,  27,  27,  27,  27,  26,  26,  26,  26,  26,  26,  25,  25,  25,  25,  25,  25,  24,  24,  24,  24,  24,  24,  23,  23,  23,  23,  23,  23,  23,  22,  22,  22,  22,  22,  22,  22,  22,  21,  21,  21,  21,  21,  21,  21,  21,  21,  20,  20,  20,  20,  20,  20,  20,  20,  20,  19,  19,  19,  19,  19,  19,  19,  19,  19,  19,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  17,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  16,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15,  15  };


   #endif