{ stdenv
, mkRosPackage
, robonomics_comm
, python3Packages
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "uruguay_collector_server";
  version = "0.1.0";

  src = ./.;

  propagatedBuildInputs = [
    robonomics_comm
    python3Packages.psycopg2
    python3Packages.sqlalchemy
  ];

  meta = with stdenv.lib; {
    description = "Agent accepts data from Uruguay";
    homepage = http://github.com/vourhey/uruguay_collector_server;
    license = licenses.bsd3;
    maintainers = with maintainers; [ vourhey ];
  };
}
