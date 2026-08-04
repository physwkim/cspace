#!/usr/bin/env perl
# Usage: tools/ci/count-relative-eq.pl <file.rs> [<file.rs> ...]
#
# The single copy for the whole workspace. It used to live in
# `crates/moveit-geometry/audit/` and was copied into four more crates, which
# is how the divergence this file exists to prevent actually happened: the
# `tools/moveit-diff/` copy never picked up the block-comment and
# string-literal fixes below, so the same command gave two different
# classifications depending on which crate you ran it from. Do not copy it
# back into a crate -- run it from here, with an explicit file list.
#
# Those fixes came from running the script against its own `audit/` directory
# and against a synthetic block-comment/string-literal fixture: it counted its
# own doc-comment example as two live calls (its `#` Perl-comment lines are
# not `//`-stripped) and counted a fake call written inside a `/* */` block
# comment or a `"..."` string literal as real. Neither false positive changed
# the geometry/octomap counts taken before the fix, nor p1-joints'
# `moveit-kinematics`/`moveit-diff`/`invariants.rs` count, which was taken
# with the unfixed copy and re-run against this one at consolidation time --
# `both=0 epsilon_only=2 max_relative_only=0 neither=0` either way.
#
# Strips `/* */` block comments (including multi-line) and `//` line-comment
# tails, then blanks the contents of every `"..."` string literal (handles
# `\"` escapes; this repo's real call sites use no raw (`r"..."`) or byte
# (`b"..."`) string literals, confirmed by `rg`, so those forms are not
# handled), then finds every remaining `assert_relative_eq!(` /
# `relative_eq!(` call by bracket-matching parens from the macro name to the
# matching close, and classifies each by whether `epsilon =` and/or
# `max_relative =` appear inside that call's own argument text.
use strict;
use warnings;
my ($both, $eps_only, $mr_only, $neither) = (0,0,0,0);
for my $file (@ARGV) {
    open(my $fh, '<', $file) or die "$file: $!";
    local $/;
    my $text = <$fh>;
    close $fh;
    $text =~ s{/\*.*?\*/}{}gs;
    $text =~ s{//[^\n]*}{}g;
    $text =~ s{"(?:[^"\\]|\\.)*"}{""}g;
    while ($text =~ /\b(?:assert_relative_eq|relative_eq)!\s*\(/g) {
        my $open = rindex($text, '(', pos($text) - 1);
        my $depth = 0;
        my $j = $open;
        my $len = length($text);
        for (; $j < $len; $j++) {
            my $c = substr($text, $j, 1);
            $depth++ if $c eq '(';
            $depth-- if $c eq ')';
            last if $depth == 0;
        }
        my $call = substr($text, $open, $j - $open + 1);
        my $has_eps = $call =~ /\bepsilon\s*=/;
        my $has_mr  = $call =~ /\bmax_relative\s*=/;
        if ($has_eps && $has_mr) { $both++ }
        elsif ($has_eps) { $eps_only++ }
        elsif ($has_mr) { $mr_only++ }
        else { $neither++ }
        printf STDERR "%s: call at byte %d -> eps=%d mr=%d\n", $file, $open, $has_eps, $has_mr;
    }
}
print "both=$both epsilon_only=$eps_only max_relative_only=$mr_only neither=$neither\n";
