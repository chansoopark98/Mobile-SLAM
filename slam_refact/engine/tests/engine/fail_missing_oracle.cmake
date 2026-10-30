# Run by ctest (cmake -P) when the evidence root exists but the api-oracle extracts of RM-3a / US-002..005
# are missing: the XrController API evidence tests must fail loudly, not disappear from the test list.
message(FATAL_ERROR
  "BLOCKED: XrController API evidence tests are not registered because ${ORACLE_DIR} does not "
  "contain tsv-api1/hits.tsv, tsv-api3/lighting.tsv, or tsv-ab1..3/hit_ab.tsv for the RM-3b hit A/B "
  "oracle in the sibling oracle-ab directory (rebuild them with "
  "slam_refact/tools/harness/oracle/extract_api_oracle.py, extract_lighting_oracle.py and "
  "extract_hit_ab.py, oracle runs from slam_refact/tools/re/runtime/run-reference.mjs --api-oracle). A missing oracle "
  "directory is a failure, not a skip.")
