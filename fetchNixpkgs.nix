{ rev    ? "76f00209d110e21367d14172d4877dcd536701b0"             # The Git revision of nixpkgs to fetch
, sha256 ? "18zh59rn77pxvkf0pjndjvcrhk2wh6dn9yx38q2izn86364n9g09" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
