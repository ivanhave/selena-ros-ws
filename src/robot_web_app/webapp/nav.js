(function () {
    var hamburger = document.getElementById('hamburger');
    var drawer    = document.getElementById('nav-drawer');
    var overlay   = document.getElementById('nav-overlay');
    if (!hamburger || !drawer || !overlay) return;

    function openDrawer() {
        drawer.classList.add('nav-drawer--open');
        overlay.classList.add('nav-overlay--visible');
    }

    function closeDrawer() {
        drawer.classList.remove('nav-drawer--open');
        overlay.classList.remove('nav-overlay--visible');
    }

    hamburger.addEventListener('click', openDrawer);
    hamburger.addEventListener('touchend', function (e) {
        e.preventDefault();
        openDrawer();
    });

    overlay.addEventListener('click', closeDrawer);
    overlay.addEventListener('touchend', function (e) {
        e.preventDefault();
        closeDrawer();
    });

    drawer.querySelectorAll('a').forEach(function (a) {
        a.addEventListener('click', closeDrawer);
    });
})();
